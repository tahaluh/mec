import numpy as np
from typing import List, Tuple
import json

class TrussElement:
    def __init__(self, node1: int, node2: int, E: float, A: float, L: float = None):
        """
        Inicializa um elemento de treliça
        node1, node2: índices dos nós que formam o elemento
        E: Módulo de elasticidade (MPa)
        A: Área da seção transversal (mm²)
        L: Comprimento do elemento (mm), opcional - será calculado se não fornecido
        """
        self.node1 = node1
        self.node2 = node2
        self.E = E
        self.A = A
        self.L = L
        self.cos = None  # cosseno do ângulo com o eixo x
        self.sin = None  # seno do ângulo com o eixo x

    def calculate_properties(self, nodes: List[Tuple[float, float]]):
        """
        Calcula as propriedades geométricas do elemento
        nodes: lista de tuplas (x,y) com as coordenadas dos nós
        """
        # Coordenadas dos nós
        x1, y1 = nodes[self.node1]
        x2, y2 = nodes[self.node2]
        
        # Calcula comprimento se não fornecido
        dx = x2 - x1
        dy = y2 - y1
        self.L = np.sqrt(dx**2 + dy**2) if self.L is None else self.L
        
        # Calcula seno e cosseno (direção do elemento)
        self.cos = dx/self.L  # cos = adj/hip = dx/L
        self.sin = dy/self.L  # sin = op/hip = dy/L
        
        # Retorna a matriz de rigidez do elemento no sistema global
        EA_L = (self.E * self.A) / self.L
        c2 = self.cos**2
        s2 = self.sin**2
        cs = self.cos * self.sin
        
        # Matriz de rigidez no sistema global [k]{u} = {f}
        return EA_L * np.array([
            [ c2,  cs, -c2, -cs],
            [ cs,  s2, -cs, -s2],
            [-c2, -cs,  c2,  cs],
            [-cs, -s2,  cs,  s2]
        ])

    def to_dict(self):
        return {
            'node1': self.node1,
            'node2': self.node2,
            'E': self.E,
            'A': self.A,
            'L': self.L
        }

    @classmethod
    def from_dict(cls, data):
        # L parameter is optional
        L = data.get('L', None)
        return cls(data['node1'], data['node2'], data['E'], data['A'], L)

class TrussStructure:
    def __init__(self):
        """Inicializa uma estrutura de treliça vazia"""
        self.nodes = []         # Lista de coordenadas (x,y) dos nós
        self.elements = []      # Lista de elementos
        self.fixed_dofs = set() # Conjunto de graus de liberdade fixos
        self.loads = {}         # Dicionário de cargas {gdl: força}
        self.displacements = None
        self.forces = None

    def add_node(self, x: float, y: float) -> int:
        """Adiciona um nó nas coordenadas (x,y)"""
        self.nodes.append((x, y))
        return len(self.nodes) - 1

    def add_element(self, node1: int, node2: int, E: float, A: float, L: float = None) -> int:
        """Adiciona um elemento entre os nós node1 e node2"""
        element = TrussElement(node1, node2, E, A, L)
        self.elements.append(element)
        return len(self.elements) - 1

    def fix_node(self, node: int, fix_x: bool = True, fix_y: bool = True):
        """Restringe os graus de liberdade de um nó"""
        if fix_x:
            self.fixed_dofs.add(2 * node)     # Restringe deslocamento em x
        if fix_y:
            self.fixed_dofs.add(2 * node + 1) # Restringe deslocamento em y

    def apply_load(self, node: int, fx: float = 0, fy: float = 0):
        """Aplica forças em um nó"""
        if fx != 0:
            self.loads[2 * node] = fx     # Força em x
        if fy != 0:
            self.loads[2 * node + 1] = fy # Força em y

    def solve(self):
        """
        Resolve o sistema de equações da treliça
        Retorna: (deslocamentos, forças_elementos)
        """
        n_nodes = len(self.nodes)
        n_dofs = 2 * n_nodes  # Cada nó tem 2 graus de liberdade (x,y)
        
        # Inicializa a matriz de rigidez global
        K = np.zeros((n_dofs, n_dofs))
        
        # Monta a matriz de rigidez global
        for element in self.elements:
            # Calcula a matriz de rigidez do elemento
            k_e = element.calculate_properties(self.nodes)
            
            # Índices dos graus de liberdade do elemento
            dofs = [
                2*element.node1,    # u1 (x1)
                2*element.node1+1,  # v1 (y1)
                2*element.node2,    # u2 (x2)
                2*element.node2+1   # v2 (y2)
            ]
            
            # Adiciona a contribuição do elemento à matriz global
            for i in range(4):
                for j in range(4):
                    K[dofs[i], dofs[j]] += k_e[i, j]
        
        # Monta o vetor de forças global
        F = np.zeros(n_dofs)
        for dof, force in self.loads.items():
            F[dof] = force
        
        # Aplica as condições de contorno
        free_dofs = [i for i in range(n_dofs) if i not in self.fixed_dofs]
        K_reduced = K[np.ix_(free_dofs, free_dofs)]
        F_reduced = F[free_dofs]
        
        # Resolve o sistema KU = F
        U_reduced = np.linalg.solve(K_reduced, F_reduced)
        
        # Monta o vetor de deslocamentos completo
        U = np.zeros(n_dofs)
        for i, dof in enumerate(free_dofs):
            U[dof] = U_reduced[i]
        
        self.displacements = U
        
        # Calcula as forças nos elementos
        element_forces = []
        for element in self.elements:
            # Vetor de deslocamentos do elemento
            u_e = np.array([
                U[2*element.node1],    # u1 (x1)
                U[2*element.node1+1],  # v1 (y1)
                U[2*element.node2],    # u2 (x2)
                U[2*element.node2+1]   # v2 (y2)
            ])
            
            # Calcula a força axial: F = (EA/L)[-c -s c s]{u}
            EA_L = element.E * element.A / element.L
            c = element.cos
            s = element.sin
            force = EA_L * (-c * u_e[0] - s * u_e[1] + c * u_e[2] + s * u_e[3])
            element_forces.append(force)
        
        self.forces = element_forces
        return U, element_forces

    def get_element_info(self):
        """Get information about each element"""
        info = []
        for i, element in enumerate(self.elements):
            force = self.forces[i]
            force_type = "Tração" if force > 0 else "Compressão"
            info.append({
                'element': i,
                'force': force,
                'force_type': force_type,
                'length': element.L,
                'cos': element.cos,
                'sin': element.sin
            })
        return info

    def save_to_file(self, filename: str):
        """Save the current state to a JSON file"""
        data = {
            'nodes': self.nodes,
            'elements': [elem.to_dict() for elem in self.elements],
            'fixed_dofs': list(self.fixed_dofs),
            'loads': {str(k): v for k, v in self.loads.items()}  # Convert keys to strings for JSON
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)

    def load_from_file(self, filename: str):
        """Load state from a JSON file"""
        with open(filename, 'r') as f:
            data = json.load(f)
        
        # Clear current state
        self.nodes = []
        self.elements = []
        self.fixed_dofs = set()
        self.loads = {}
        
        # Load new state
        self.nodes = [tuple(node) for node in data['nodes']]  # Convert node lists to tuples
        self.elements = [TrussElement.from_dict(elem) for elem in data['elements']]
        self.fixed_dofs = set(data['fixed_dofs'])
        
        # Handle loads - convert keys to integers if they're strings
        loads_data = data['loads']
        self.loads = {int(k): float(v) for k, v in loads_data.items()} 