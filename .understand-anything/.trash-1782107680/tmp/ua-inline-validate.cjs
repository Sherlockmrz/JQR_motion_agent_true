const fs=require('fs');
const graph=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
const issues=[]; const warnings=[];
const ids=new Set(graph.nodes.map(n=>n.id));
for(const [i,n] of graph.nodes.entries()){
  if(!n.id) issues.push(`Node[${i}] missing id`);
  if(!n.type) issues.push(`Node ${n.id} missing type`);
  if(!n.name) issues.push(`Node ${n.id} missing name`);
  if(!n.summary) issues.push(`Node ${n.id} missing summary`);
  if(!Array.isArray(n.tags)||!n.tags.length) issues.push(`Node ${n.id} missing tags`);
}
for(const [i,e] of graph.edges.entries()){
  if(!ids.has(e.source)) issues.push(`Edge[${i}] source missing ${e.source}`);
  if(!ids.has(e.target)) issues.push(`Edge[${i}] target missing ${e.target}`);
}
const assigned=new Map();
for(const layer of graph.layers||[]){
  if(!layer.id||!layer.name||!layer.description||!Array.isArray(layer.nodeIds)) issues.push(`Bad layer ${layer.id||'?'} `);
  for(const id of layer.nodeIds||[]){
    if(!ids.has(id)) issues.push(`Layer ${layer.id} missing node ${id}`);
    if(assigned.has(id)) issues.push(`Node ${id} in multiple layers`);
    assigned.set(id,layer.id);
  }
}
for(const n of graph.nodes.filter(n=>['file','config','document','service','pipeline','table','schema','resource','endpoint'].includes(n.type))){
  if(!assigned.has(n.id)) issues.push(`File-level node ${n.id} not in layer`);
}
for(const [i,step] of (graph.tour||[]).entries()){
  if(!step.order||!step.title||!step.description||!Array.isArray(step.nodeIds)) issues.push(`Bad tour step ${i}`);
  for(const id of step.nodeIds||[]) if(!ids.has(id)) issues.push(`Tour step ${i} missing node ${id}`);
}
const stats={totalNodes:graph.nodes.length,totalEdges:graph.edges.length,totalLayers:graph.layers.length,tourSteps:graph.tour.length,nodeTypes:{},edgeTypes:{}};
for(const n of graph.nodes) stats.nodeTypes[n.type]=(stats.nodeTypes[n.type]||0)+1;
for(const e of graph.edges) stats.edgeTypes[e.type]=(stats.edgeTypes[e.type]||0)+1;
fs.writeFileSync(process.argv[3], JSON.stringify({issues,warnings,stats}, null, 2));
if(issues.length) process.exitCode=2;