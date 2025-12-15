#!/usr/bin/env python3
"""
Action Graph Analyzer for USD/OmniGraph
========================================
Analisa a estrutura de um Action Graph em um arquivo USD.

Uso:
    python analyze_action_graph.py [--stage PATH] [--graph GRAPH_PATH]
    
Exemplo:
    python analyze_action_graph.py --stage /workspace/zed_streamer_warehouse.usd --graph /Root/ActionGraph_02
"""

import argparse
import sys
import os
from typing import List, Dict, Any, Optional
from collections import defaultdict

# Configurar ambiente para USD/pxr do Isaac Sim
def setup_isaac_sim_env():
    """Configura o ambiente para usar pxr do Isaac Sim"""
    isaac_sim_path = "/isaac-sim"
    usd_libs_path = None
    
    # Encontrar o caminho do omni.usd.libs
    extscache = os.path.join(isaac_sim_path, "extscache")
    if os.path.exists(extscache):
        for d in os.listdir(extscache):
            if d.startswith("omni.usd.libs"):
                usd_libs_path = os.path.join(extscache, d)
                break
    
    if usd_libs_path and os.path.exists(usd_libs_path):
        # Adicionar ao PYTHONPATH
        if usd_libs_path not in sys.path:
            sys.path.insert(0, usd_libs_path)
        
        # Adicionar bibliotecas ao LD_LIBRARY_PATH
        bin_path = os.path.join(usd_libs_path, "bin")
        if os.path.exists(bin_path):
            current_ld = os.environ.get("LD_LIBRARY_PATH", "")
            if bin_path not in current_ld:
                os.environ["LD_LIBRARY_PATH"] = f"{bin_path}:{current_ld}"
        
        return True
    return False

# Tentar configurar o ambiente Isaac Sim
setup_isaac_sim_env()

try:
    from pxr import Usd, UsdGeom, Sdf, Gf
except ImportError as e:
    print(f"❌ Erro: pxr (USD) não encontrado: {e}")
    print("\n💡 Execute o script de uma das seguintes formas:")
    print("   1. Com wrapper script:")
    print("      /workspace/scripts/run_usd_analyzer.sh")
    print("   2. Com variáveis de ambiente:")
    print("      export LD_LIBRARY_PATH=/isaac-sim/extscache/omni.usd.libs-1.0.1+8131b85d.lx64.r.cp311/bin:$LD_LIBRARY_PATH")
    print("      export PYTHONPATH=/isaac-sim/extscache/omni.usd.libs-1.0.1+8131b85d.lx64.r.cp311:$PYTHONPATH")
    print("      /isaac-sim/kit/python/bin/python3 analyze_action_graph.py")
    sys.exit(1)


class Colors:
    """ANSI color codes para terminal"""
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'


class ActionGraphAnalyzer:
    """Analisador de Action Graph para USD"""
    
    def __init__(self, stage_path: str, graph_path: str = "/Root/ActionGraph_02"):
        self.stage_path = stage_path
        self.graph_path = graph_path
        self.stage: Optional[Usd.Stage] = None
        self.graph_prim: Optional[Usd.Prim] = None
        self.nodes: List[Usd.Prim] = []
        self.connections: List[Dict[str, Any]] = []
        
    def load_stage(self) -> bool:
        """Carrega o stage USD"""
        print(f"\n{Colors.HEADER}{'='*60}{Colors.END}")
        print(f"{Colors.BOLD}📂 Carregando Stage USD{Colors.END}")
        print(f"{Colors.HEADER}{'='*60}{Colors.END}")
        
        try:
            self.stage = Usd.Stage.Open(self.stage_path)
            if not self.stage:
                print(f"{Colors.RED}❌ Falha ao abrir stage: {self.stage_path}{Colors.END}")
                return False
            print(f"{Colors.GREEN}✅ Stage carregado: {self.stage_path}{Colors.END}")
            return True
        except Exception as e:
            print(f"{Colors.RED}❌ Erro: {e}{Colors.END}")
            return False
    
    def find_action_graph(self) -> bool:
        """Encontra o Action Graph especificado"""
        print(f"\n{Colors.CYAN}🔍 Procurando Action Graph: {self.graph_path}{Colors.END}")
        
        self.graph_prim = self.stage.GetPrimAtPath(self.graph_path)
        
        if not self.graph_prim or not self.graph_prim.IsValid():
            print(f"{Colors.YELLOW}⚠️  Graph não encontrado em: {self.graph_path}{Colors.END}")
            print(f"\n{Colors.CYAN}📋 Procurando graphs disponíveis...{Colors.END}")
            self._list_available_graphs()
            return False
        
        print(f"{Colors.GREEN}✅ Action Graph encontrado!{Colors.END}")
        print(f"   Tipo: {self.graph_prim.GetTypeName()}")
        return True
    
    def _list_available_graphs(self):
        """Lista todos os graphs disponíveis no stage"""
        graphs_found = []
        for prim in self.stage.Traverse():
            type_name = prim.GetTypeName()
            path = str(prim.GetPath())
            
            # Identifica potenciais Action Graphs
            if any(keyword in type_name.lower() for keyword in ['graph', 'omnigraph', 'action']):
                graphs_found.append((path, type_name))
            elif 'ActionGraph' in path or 'OmniGraph' in path:
                graphs_found.append((path, type_name))
        
        if graphs_found:
            print(f"\n{Colors.YELLOW}📊 Graphs disponíveis:{Colors.END}")
            for path, type_name in graphs_found:
                print(f"   • {path} ({type_name})")
        else:
            print(f"{Colors.RED}   Nenhum graph encontrado no stage.{Colors.END}")
    
    def analyze_nodes(self):
        """Analisa todos os nós do Action Graph"""
        print(f"\n{Colors.HEADER}{'='*60}{Colors.END}")
        print(f"{Colors.BOLD}🔧 Análise de Nós do Action Graph{Colors.END}")
        print(f"{Colors.HEADER}{'='*60}{Colors.END}")
        
        self.nodes = []
        node_types = defaultdict(int)
        
        # Coleta nós filhos do graph
        def collect_nodes(prim: Usd.Prim, depth: int = 0):
            for child in prim.GetChildren():
                self.nodes.append((child, depth))
                node_types[child.GetTypeName()] += 1
                collect_nodes(child, depth + 1)
        
        collect_nodes(self.graph_prim)
        
        print(f"\n{Colors.CYAN}📈 Estatísticas:{Colors.END}")
        print(f"   Total de nós: {len(self.nodes)}")
        print(f"\n{Colors.CYAN}📊 Tipos de nós:{Colors.END}")
        for node_type, count in sorted(node_types.items(), key=lambda x: -x[1]):
            print(f"   • {node_type}: {count}")
        
        print(f"\n{Colors.CYAN}🗂️  Hierarquia de Nós:{Colors.END}")
        for node, depth in self.nodes:
            indent = "   " + "  │ " * depth
            type_color = Colors.GREEN if "Ros" in node.GetTypeName() else Colors.BLUE
            print(f"{indent}├─ {Colors.BOLD}{node.GetName()}{Colors.END} {type_color}({node.GetTypeName()}){Colors.END}")
    
    def analyze_node_details(self, max_nodes: int = 50):
        """Analisa detalhes de cada nó"""
        print(f"\n{Colors.HEADER}{'='*60}{Colors.END}")
        print(f"{Colors.BOLD}📝 Detalhes dos Nós{Colors.END}")
        print(f"{Colors.HEADER}{'='*60}{Colors.END}")
        
        nodes_to_analyze = self.nodes[:max_nodes]
        
        for node, _ in nodes_to_analyze:
            self._print_node_details(node)
    
    def _print_node_details(self, node: Usd.Prim):
        """Imprime detalhes de um nó específico"""
        print(f"\n{Colors.YELLOW}┌{'─'*58}┐{Colors.END}")
        print(f"{Colors.YELLOW}│{Colors.END} {Colors.BOLD}📦 {node.GetName()}{Colors.END}")
        print(f"{Colors.YELLOW}│{Colors.END}    Path: {node.GetPath()}")
        print(f"{Colors.YELLOW}│{Colors.END}    Type: {Colors.CYAN}{node.GetTypeName()}{Colors.END}")
        print(f"{Colors.YELLOW}└{'─'*58}┘{Colors.END}")
        
        # Atributos
        attributes = node.GetAttributes()
        inputs = []
        outputs = []
        other_attrs = []
        
        for attr in attributes:
            name = attr.GetName()
            if name.startswith("inputs:"):
                inputs.append(attr)
            elif name.startswith("outputs:"):
                outputs.append(attr)
            elif attr.HasValue():
                other_attrs.append(attr)
        
        # Inputs
        if inputs:
            print(f"   {Colors.GREEN}📥 Inputs:{Colors.END}")
            for attr in inputs:
                self._print_attribute(attr, "      ")
        
        # Outputs
        if outputs:
            print(f"   {Colors.BLUE}📤 Outputs:{Colors.END}")
            for attr in outputs:
                self._print_attribute(attr, "      ")
        
        # Conexões
        connections = node.GetRelationships()
        if connections:
            print(f"   {Colors.CYAN}🔗 Conexões:{Colors.END}")
            for rel in connections:
                targets = rel.GetTargets()
                if targets:
                    print(f"      • {rel.GetName()}:")
                    for target in targets:
                        print(f"        → {target}")
        
        # Outros atributos importantes
        important_attrs = [a for a in other_attrs if self._is_important_attr(a)]
        if important_attrs:
            print(f"   {Colors.YELLOW}⚙️  Configurações:{Colors.END}")
            for attr in important_attrs[:10]:  # Limita a 10
                self._print_attribute(attr, "      ")
    
    def _print_attribute(self, attr, indent: str = ""):
        """Imprime um atributo formatado"""
        name = attr.GetName()
        try:
            value = attr.Get()
            value_str = self._format_value(value)
            type_name = attr.GetTypeName()
            
            # Detecta conexões
            connections = attr.GetConnections()
            if connections:
                print(f"{indent}• {name} [{type_name}]:")
                for conn in connections:
                    print(f"{indent}  {Colors.CYAN}⟵ connected from: {conn}{Colors.END}")
            elif value is not None:
                print(f"{indent}• {name} [{type_name}]: {value_str}")
        except Exception as e:
            print(f"{indent}• {name}: <erro ao ler: {e}>")
    
    def _format_value(self, value, max_length: int = 80) -> str:
        """Formata valor para exibição"""
        if value is None:
            return "None"
        
        value_str = str(value)
        if len(value_str) > max_length:
            return value_str[:max_length] + "..."
        return value_str
    
    def _is_important_attr(self, attr) -> bool:
        """Determina se um atributo é importante para exibição"""
        name = attr.GetName().lower()
        important_keywords = [
            'topic', 'frame', 'target', 'prim', 'path', 'type', 
            'enabled', 'node', 'queue', 'namespace', 'robot',
            'joint', 'controller', 'command', 'message'
        ]
        return any(kw in name for kw in important_keywords)
    
    def analyze_connections(self):
        """Analisa conexões entre nós"""
        print(f"\n{Colors.HEADER}{'='*60}{Colors.END}")
        print(f"{Colors.BOLD}🔗 Mapa de Conexões{Colors.END}")
        print(f"{Colors.HEADER}{'='*60}{Colors.END}")
        
        connection_map = defaultdict(list)
        
        for node, _ in self.nodes:
            node_path = str(node.GetPath())
            
            # Busca conexões em atributos
            for attr in node.GetAttributes():
                connections = attr.GetConnections()
                for conn in connections:
                    source_path = str(conn)
                    connection_map[source_path].append({
                        'target': f"{node_path}.{attr.GetName()}",
                        'attr_name': attr.GetName()
                    })
            
            # Busca conexões em relacionamentos
            for rel in node.GetRelationships():
                for target in rel.GetTargets():
                    connection_map[node_path].append({
                        'target': str(target),
                        'rel_name': rel.GetName()
                    })
        
        if connection_map:
            print(f"\n{Colors.CYAN}Conexões encontradas:{Colors.END}")
            for source, targets in connection_map.items():
                print(f"\n   {Colors.GREEN}📤 {source}{Colors.END}")
                for t in targets:
                    if 'attr_name' in t:
                        print(f"      └─→ {t['target']}")
                    else:
                        print(f"      └─→ [{t['rel_name']}] {t['target']}")
        else:
            print(f"   {Colors.YELLOW}Nenhuma conexão explícita encontrada.{Colors.END}")
    
    def find_ros_nodes(self):
        """Encontra e analisa nós ROS especificamente"""
        print(f"\n{Colors.HEADER}{'='*60}{Colors.END}")
        print(f"{Colors.BOLD}🤖 Nós ROS/ROS2{Colors.END}")
        print(f"{Colors.HEADER}{'='*60}{Colors.END}")
        
        ros_nodes = []
        for node, depth in self.nodes:
            type_name = node.GetTypeName()
            if 'Ros' in type_name or 'ros' in type_name.lower():
                ros_nodes.append(node)
        
        if ros_nodes:
            print(f"\n   {Colors.GREEN}Encontrados {len(ros_nodes)} nós ROS:{Colors.END}")
            for node in ros_nodes:
                print(f"\n   {Colors.CYAN}• {node.GetName()}{Colors.END} ({node.GetTypeName()})")
                
                # Busca topic names
                for attr in node.GetAttributes():
                    name = attr.GetName().lower()
                    if 'topic' in name or 'frame' in name:
                        try:
                            value = attr.Get()
                            print(f"     {attr.GetName()}: {value}")
                        except:
                            pass
        else:
            print(f"   {Colors.YELLOW}Nenhum nó ROS encontrado.{Colors.END}")
    
    def generate_summary(self):
        """Gera um resumo da análise"""
        print(f"\n{Colors.HEADER}{'='*60}{Colors.END}")
        print(f"{Colors.BOLD}📋 RESUMO DA ANÁLISE{Colors.END}")
        print(f"{Colors.HEADER}{'='*60}{Colors.END}")
        
        print(f"""
   {Colors.CYAN}Stage:{Colors.END} {self.stage_path}
   {Colors.CYAN}Graph:{Colors.END} {self.graph_path}
   {Colors.CYAN}Total de Nós:{Colors.END} {len(self.nodes)}
   
   {Colors.CYAN}Tipos de Nós:{Colors.END}
""")
        
        type_count = defaultdict(int)
        for node, _ in self.nodes:
            type_count[node.GetTypeName()] += 1
        
        for t, c in sorted(type_count.items(), key=lambda x: -x[1])[:10]:
            bar = '█' * min(c, 20)
            print(f"      {t}: {bar} ({c})")
        
        print(f"\n{Colors.GREEN}✅ Análise concluída!{Colors.END}\n")
    
    def run_full_analysis(self):
        """Executa análise completa"""
        if not self.load_stage():
            return False
        
        if not self.find_action_graph():
            return False
        
        self.analyze_nodes()
        self.analyze_node_details()
        self.analyze_connections()
        self.find_ros_nodes()
        self.generate_summary()
        
        return True


def search_all_graphs(stage_path: str):
    """Busca todos os Action Graphs em um stage"""
    print(f"\n{Colors.HEADER}{'='*60}{Colors.END}")
    print(f"{Colors.BOLD}🔍 Buscando todos os Action Graphs{Colors.END}")
    print(f"{Colors.HEADER}{'='*60}{Colors.END}")
    
    try:
        stage = Usd.Stage.Open(stage_path)
        if not stage:
            print(f"{Colors.RED}❌ Falha ao abrir stage{Colors.END}")
            return []
        
        graphs = []
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            type_name = prim.GetTypeName()
            
            # Identifica graphs por tipo ou nome
            is_graph = (
                'Graph' in type_name or
                'ActionGraph' in path or
                'OmniGraph' in path or
                type_name == 'OmniGraphSchema' or
                'omni.graph' in type_name.lower()
            )
            
            if is_graph:
                graphs.append({
                    'path': path,
                    'type': type_name,
                    'prim': prim
                })
        
        print(f"\n{Colors.CYAN}Encontrados {len(graphs)} graphs:{Colors.END}")
        for i, g in enumerate(graphs, 1):
            print(f"   {i}. {g['path']} ({g['type']})")
        
        return graphs
        
    except Exception as e:
        print(f"{Colors.RED}❌ Erro: {e}{Colors.END}")
        return []


def main():
    parser = argparse.ArgumentParser(
        description='Analisa Action Graphs em arquivos USD',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemplos:
  # Analisa graph específico
  python analyze_action_graph.py --stage scene.usd --graph /Root/ActionGraph_02
  
  # Lista todos os graphs disponíveis
  python analyze_action_graph.py --stage scene.usd --list-graphs
  
  # Usa valores padrão
  python analyze_action_graph.py
        """
    )
    
    parser.add_argument(
        '--stage', '-s',
        default='/workspace/zed_streamer_warehouse.usd',
        help='Caminho para o arquivo USD (padrão: zed_streamer_warehouse.usd)'
    )
    
    parser.add_argument(
        '--graph', '-g',
        default='/Root/ActionGraph_02',
        help='Caminho do Action Graph no stage (padrão: /Root/ActionGraph_02)'
    )
    
    parser.add_argument(
        '--list-graphs', '-l',
        action='store_true',
        help='Lista todos os graphs disponíveis no stage'
    )
    
    parser.add_argument(
        '--max-nodes', '-m',
        type=int,
        default=50,
        help='Número máximo de nós para análise detalhada (padrão: 50)'
    )
    
    args = parser.parse_args()
    
    print(f"""
{Colors.HEADER}╔══════════════════════════════════════════════════════════╗
║         ACTION GRAPH ANALYZER for USD/OmniGraph          ║
╚══════════════════════════════════════════════════════════╝{Colors.END}
    """)
    
    if args.list_graphs:
        search_all_graphs(args.stage)
        return
    
    analyzer = ActionGraphAnalyzer(args.stage, args.graph)
    success = analyzer.run_full_analysis()
    
    if not success:
        print(f"\n{Colors.YELLOW}💡 Dica: Use --list-graphs para ver graphs disponíveis{Colors.END}")
        sys.exit(1)


if __name__ == "__main__":
    main()
