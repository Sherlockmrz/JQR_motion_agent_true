const fs = require('fs');
const path = require('path');
const childProcess = require('child_process');

const root = process.argv[2] || process.cwd();
const ua = path.join(root, '.understand-anything');
const inter = path.join(ua, 'intermediate');

function readJson(file) {
  return JSON.parse(fs.readFileSync(file, 'utf8').replace(/^\uFEFF/, ''));
}

const scan = readJson(path.join(inter, 'scan-result.json'));
const graph = readJson(path.join(inter, 'assembled-graph.json'));
const commit = childProcess.spawnSync('git', ['rev-parse', 'HEAD'], {
  cwd: root,
  encoding: 'utf8',
}).stdout.trim();

const fingerprintInput = {
  projectRoot: root,
  sourceFilePaths: scan.files.map(file => file.path),
  gitCommitHash: commit,
};
fs.writeFileSync(
  path.join(inter, 'fingerprint-input.json'),
  JSON.stringify(fingerprintInput, null, 2),
  'utf8',
);

graph.project.gitCommitHash = commit;
fs.writeFileSync(path.join(ua, 'knowledge-graph.json'), JSON.stringify(graph, null, 2), 'utf8');

const meta = {
  lastAnalyzedAt: new Date().toISOString(),
  gitCommitHash: commit,
  version: '1.0.0',
  analyzedFiles: scan.totalFiles,
};
fs.writeFileSync(path.join(ua, 'meta.json'), JSON.stringify(meta, null, 2), 'utf8');

const categoryCounts = scan.stats.byCategory;
const languageCounts = scan.stats.byLanguage;
const nodeTypes = {};
const edgeTypes = {};
for (const node of graph.nodes) nodeTypes[node.type] = (nodeTypes[node.type] || 0) + 1;
for (const edge of graph.edges) edgeTypes[edge.type] = (edgeTypes[edge.type] || 0) + 1;

console.log(JSON.stringify({
  files: scan.totalFiles,
  filteredByIgnore: scan.filteredByIgnore,
  categories: categoryCounts,
  languages: languageCounts,
  nodes: graph.nodes.length,
  edges: graph.edges.length,
  nodeTypes,
  edgeTypes,
  layers: graph.layers.map(layer => layer.name),
  tourSteps: graph.tour.length,
  output: path.join(ua, 'knowledge-graph.json'),
}, null, 2));
