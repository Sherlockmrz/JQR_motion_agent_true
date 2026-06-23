const fs = require('fs');
const path = require('path');
const childProcess = require('child_process');

const root = process.argv[2] || process.cwd();
const inter = path.join(root, '.understand-anything', 'intermediate');
const out = path.join(inter, 'assembled-graph.json');

function readJson(file) {
  return JSON.parse(fs.readFileSync(file, 'utf8').replace(/^\uFEFF/, ''));
}

function posix(p) {
  return p.split(path.sep).join('/');
}

function nodeTypeFor(file) {
  if (file.fileCategory === 'docs') return 'document';
  if (file.fileCategory === 'config') return 'config';
  if (file.fileCategory === 'infra') return 'service';
  if (file.fileCategory === 'data') return 'schema';
  if (file.path.endsWith('.service')) return 'service';
  return 'file';
}

function fileNodeId(file) {
  return `${nodeTypeFor(file)}:${file.path}`;
}

function functionId(filePath, name) {
  return `function:${filePath}:${name}`;
}

function classId(filePath, name) {
  return `class:${filePath}:${name}`;
}

function categoryText(category) {
  return {
    code: '代码文件',
    config: '配置文件',
    docs: '文档文件',
    infra: '基础设施文件',
    data: '数据或 schema 文件',
    script: '脚本文件',
    markup: '界面标记文件',
  }[category] || '项目文件';
}

function fileSummary(file, result) {
  const parts = [`${file.path} 是一个 ${categoryText(file.fileCategory)}`];
  if (file.language) parts.push(`语言/格式为 ${file.language}`);
  if (result?.classes?.length) parts.push(`包含 ${result.classes.length} 个类`);
  if (result?.functions?.length) parts.push(`包含 ${result.functions.length} 个函数`);
  if (result?.sections?.length) parts.push(`包含 ${result.sections.length} 个文档章节`);
  if (result?.definitions?.length) parts.push(`包含 ${result.definitions.length} 个结构定义`);
  parts.push(`约 ${file.sizeLines ?? result?.totalLines ?? 0} 行`);
  return `${parts.join('，')}。`;
}

function tagsFor(file) {
  const tags = [file.language || 'unknown', file.fileCategory || 'file'];
  const p = file.path.toLowerCase();
  if (p.includes('test') || p.includes('verify')) tags.push('test');
  if (p.includes('websocket') || p.includes('ws')) tags.push('websocket');
  if (p.includes('serial') || p.includes('protocol')) tags.push('protocol');
  if (p.includes('ros') || p.includes('msg') || p.includes('srv')) tags.push('ros2');
  if (p.includes('mock')) tags.push('mock');
  return [...new Set(tags)];
}

function edge(source, target, type, weight = 0.5, label = '') {
  return {
    id: `${type}:${source}->${target}`,
    source,
    target,
    type,
    weight,
    ...(label ? { label } : {}),
  };
}

function layerForFile(file) {
  const p = file.path.toLowerCase();
  if (p.includes('smart_robot_agent') || p === 'config.py' || p === 'agent.service') return 'core';
  if (p.includes('websocket') || p.includes('local_model_bridge') || p.endsWith('.html') || p.includes('ws')) return 'websocket';
  if (p.includes('serial') || p.includes('protocol') || p === 'rk.rules') return 'protocol';
  if (p.startsWith('jqr_ros_msgs/') || p.startsWith('nav2_msgs/')) return 'ros';
  if (p.includes('test') || p.includes('verify') || p.includes('mock')) return 'tests';
  if (file.fileCategory === 'docs' || file.fileCategory === 'script' || file.fileCategory === 'config') return 'docs';
  return 'support';
}

const scan = readJson(path.join(inter, 'scan-result.json'));
const batches = readJson(path.join(inter, 'batches.json'));
const resultsByPath = new Map();
for (const batch of batches.batches || []) {
  const batchFile = path.join(inter, `batch-${batch.batchIndex}.json`);
  if (!fs.existsSync(batchFile)) continue;
  const data = readJson(batchFile);
  for (const result of data.results || []) resultsByPath.set(result.path, result);
}

const commit = childProcess.spawnSync('git', ['rev-parse', 'HEAD'], {
  cwd: root,
  encoding: 'utf8',
}).stdout.trim();

const nodes = [];
const edges = [];
const fileIdByPath = new Map();
const functionIdByNameAndPath = new Map();

for (const file of scan.files || []) {
  const result = resultsByPath.get(file.path);
  const id = fileNodeId(file);
  fileIdByPath.set(file.path, id);
  nodes.push({
    id,
    type: nodeTypeFor(file),
    name: path.basename(file.path),
    filePath: file.path,
    summary: fileSummary(file, result),
    tags: tagsFor(file),
    complexity: file.sizeLines > 500 ? 'high' : file.sizeLines > 150 ? 'medium' : 'low',
    languageNotes: `${file.path} 在本项目中属于 ${categoryText(file.fileCategory)}，用于支撑机器人控制、测试或部署流程。`,
  });

  for (const cls of result?.classes || []) {
    const cid = classId(file.path, cls.name);
    nodes.push({
      id: cid,
      type: 'class',
      name: cls.name,
      filePath: file.path,
      summary: `类 ${cls.name} 定义在 ${file.path}，覆盖第 ${cls.startLine ?? '?'}-${cls.endLine ?? '?'} 行。`,
      tags: ['class', file.language || 'unknown'],
    });
    edges.push(edge(id, cid, 'contains', 1.0, '包含类'));
  }

  for (const fn of result?.functions || []) {
    const fid = functionId(file.path, fn.name);
    functionIdByNameAndPath.set(`${file.path}:${fn.name}`, fid);
    nodes.push({
      id: fid,
      type: 'function',
      name: fn.name,
      filePath: file.path,
      summary: `函数 ${fn.name} 定义在 ${file.path}，覆盖第 ${fn.startLine ?? '?'}-${fn.endLine ?? '?'} 行。`,
      tags: ['function', file.language || 'unknown'],
    });
    edges.push(edge(id, fid, 'contains', 1.0, '包含函数'));
  }
}

for (const [sourcePath, targets] of Object.entries(scan.importMap || {})) {
  const source = fileIdByPath.get(sourcePath);
  if (!source) continue;
  for (const targetPath of targets || []) {
    const target = fileIdByPath.get(targetPath);
    if (target) edges.push(edge(source, target, 'imports', 0.7, '导入'));
  }
}

for (const [filePath, result] of resultsByPath.entries()) {
  for (const call of result.callGraph || []) {
    const source = functionIdByNameAndPath.get(`${filePath}:${call.caller}`);
    const target =
      functionIdByNameAndPath.get(`${filePath}:${call.callee}`) ||
      [...functionIdByNameAndPath.entries()].find(([key]) => key.endsWith(`:${call.callee}`))?.[1];
    if (source && target && source !== target) edges.push(edge(source, target, 'calls', 0.8, '调用'));
  }
}

const layerDefs = {
  core: {
    id: 'layer:core-agent',
    name: '核心智能体与 ROS2 接口',
    description: '智能体主流程、配置、ROS2 动作/服务调用和系统服务入口。',
  },
  websocket: {
    id: 'layer:websocket-control',
    name: 'WebSocket 控制与模型桥接',
    description: 'WebSocket 控制服务、浏览器控制面板、本地模型桥接和相关验证入口。',
  },
  protocol: {
    id: 'layer:serial-protocol',
    name: '串口与协议解析',
    description: 'USB 串口管理、运动控制协议解析、协议 JSON 和设备规则。',
  },
  ros: {
    id: 'layer:ros-message-definitions',
    name: 'ROS 消息与服务定义',
    description: '自定义 ROS2 msg/srv/action 定义及其包配置。',
  },
  tests: {
    id: 'layer:test-and-mock',
    name: '测试与模拟节点',
    description: '端到端测试、运动控制测试、mock 节点和验证脚本。',
  },
  docs: {
    id: 'layer:docs-and-scripts',
    name: '文档、配置与运行脚本',
    description: 'README、变更记录、测试报告、启动脚本和辅助配置。',
  },
  support: {
    id: 'layer:supporting-files',
    name: '辅助文件',
    description: '无法归入主功能层的项目支持文件。',
  },
};

const layerNodeIds = Object.fromEntries(Object.keys(layerDefs).map(k => [k, []]));
for (const file of scan.files || []) {
  const key = layerForFile(file);
  layerNodeIds[key].push(fileIdByPath.get(file.path));
}
const layers = Object.entries(layerDefs)
  .map(([key, def]) => ({ ...def, nodeIds: layerNodeIds[key].filter(Boolean) }))
  .filter(layer => layer.nodeIds.length > 0);

const tour = [
  {
    order: 1,
    title: '项目入口与整体目标',
    description: '从 README 和 smart_robot_agent.py 开始，了解机器人运动代理的启动方式、版本背景和主控制流程。',
    nodeIds: ['document:README.md', 'file:smart_robot_agent.py'].filter(id => nodes.some(n => n.id === id)),
    languageLesson: '先读入口文件和 README，有助于把 ROS2、串口和 WebSocket 控制放到同一张系统图里理解。',
  },
  {
    order: 2,
    title: '控制通道',
    description: '查看 WebSocket 控制服务、本地模型桥接和控制面板，理解外部指令如何进入机器人控制层。',
    nodeIds: ['file:websocket_control_server.py', 'file:local_model_bridge.py', 'file:websocket_control_client.html'].filter(id => nodes.some(n => n.id === id)),
  },
  {
    order: 3,
    title: '串口协议链路',
    description: '沿着协议解析器和 USB 串口管理器理解运动指令如何被编码、解析和发送到设备。',
    nodeIds: ['file:protocol_parser.py', 'file:usb_serial_manager.py', 'config:combine_motor_protocol.json'].filter(id => nodes.some(n => n.id === id)),
  },
  {
    order: 4,
    title: 'ROS 消息服务边界',
    description: '查看 jqr_ros_msgs 与 nav2_msgs 下的消息、服务和 action 定义，理解系统和 ROS2 生态的接口边界。',
    nodeIds: ['file:jqr_ros_msgs/srv/MoveMode.srv', 'file:nav2_msgs/action/NavigateToPose.action', 'config:jqr_ros_msgs/package.xml'].filter(id => nodes.some(n => n.id === id)),
  },
  {
    order: 5,
    title: '测试与模拟验证',
    description: '最后浏览 mock 节点和测试脚本，理解 4DOF、航点、避障和停止指令等场景如何被验证。',
    nodeIds: ['file:mock_four_motor_node.py', 'file:test_stop_ws.py', 'file:test_four_waypoint_control.py'].filter(id => nodes.some(n => n.id === id)),
  },
].filter(step => step.nodeIds.length > 0);

const graph = {
  version: '1.0.0',
  project: {
    name: scan.projectName || path.basename(root),
    languages: scan.languages || [],
    frameworks: scan.frameworks || [],
    description: scan.projectDescription || '机器人运动代理项目。',
    analyzedAt: new Date().toISOString(),
    gitCommitHash: commit,
  },
  nodes,
  edges: [...new Map(edges.map(e => [e.id, e])).values()],
  layers,
  tour,
};

fs.writeFileSync(out, JSON.stringify(graph, null, 2), 'utf8');
console.log(`assembled graph: nodes=${graph.nodes.length} edges=${graph.edges.length} layers=${graph.layers.length} tour=${graph.tour.length}`);
