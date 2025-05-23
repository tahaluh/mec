import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                            QHBoxLayout, QPushButton, QLabel, QDoubleSpinBox,
                            QSpinBox, QComboBox, QMessageBox, QTabWidget,
                            QTableWidget, QTableWidgetItem, QHeaderView, QToolBar,
                            QFileDialog, QDialog, QDialogButtonBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIcon
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
from truss_analysis import TrussStructure

class EditableTableItem(QTableWidgetItem):
    def __init__(self, value):
        super().__init__(str(value))
        self.setTextAlignment(Qt.AlignCenter)

class TrussGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.truss = TrussStructure()
        
        self.selected_node = None
        self.selected_nodes = []
        self.zoom_level = 1.0
        self.distortion_scale = 200.0  # Default distortion scale
        self.initUI()

    def create_table_toolbar(self, table, delete_callback):
        toolbar = QToolBar()
        if delete_callback is not None:
            delete_btn = QPushButton("Deletar Selecionado")
            delete_btn.clicked.connect(delete_callback)
            toolbar.addWidget(delete_btn)
        return toolbar

    def initUI(self):
        self.setWindowTitle("Análise de Treliças")
        self.setGeometry(100, 100, 1200, 800)

        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)

        # Create left panel for controls
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        # Add Save/Load buttons
        file_group = QWidget()
        file_layout = QHBoxLayout(file_group)
        
        save_btn = QPushButton('Salvar')
        save_btn.clicked.connect(self.save_structure)
        file_layout.addWidget(save_btn)
        
        load_btn = QPushButton('Carregar')
        load_btn.clicked.connect(self.load_structure)
        file_layout.addWidget(load_btn)
        
        left_layout.addWidget(file_group)

        # Create tab widget
        self.tab_widget = QTabWidget()
        
        # Nodes tab
        nodes_tab = QWidget()
        nodes_layout = QVBoxLayout(nodes_tab)
        
        # Create nodes table first
        self.nodes_table = QTableWidget()
        self.nodes_table.setColumnCount(4)
        self.nodes_table.setHorizontalHeaderLabels(['Nó', 'X', 'Y', 'Restrições'])
        self.nodes_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.nodes_table.itemChanged.connect(self.on_node_edit)
        self.nodes_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.nodes_table.setSelectionMode(QTableWidget.ExtendedSelection)
        
        # Now create the toolbar
        nodes_toolbar = self.create_table_toolbar(self.nodes_table, self.delete_selected_nodes)
        nodes_layout.addWidget(nodes_toolbar)
        nodes_layout.addWidget(self.nodes_table)
        self.tab_widget.addTab(nodes_tab, "Nós")

        # Elements tab
        elements_tab = QWidget()
        elements_layout = QVBoxLayout(elements_tab)
        
        # Create elements table first
        self.elements_table = QTableWidget()
        self.elements_table.setColumnCount(6)
        self.elements_table.setHorizontalHeaderLabels(['Elemento', 'Nó 1', 'Nó 2', 'L [mm]', 'E [MPa]', 'A [mm²]'])
        self.elements_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.elements_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.elements_table.setSelectionMode(QTableWidget.ExtendedSelection)
        
        # Now create the toolbar
        elements_toolbar = self.create_table_toolbar(self.elements_table, self.delete_selected_elements)
        elements_layout.addWidget(elements_toolbar)
        elements_layout.addWidget(self.elements_table)
        self.tab_widget.addTab(elements_tab, "Elementos")

        # Loads tab
        loads_tab = QWidget()
        loads_layout = QVBoxLayout(loads_tab)
        
        # Create loads table first
        self.loads_table = QTableWidget()
        self.loads_table.setColumnCount(4)
        self.loads_table.setHorizontalHeaderLabels(['Nó', 'Fx', 'Fy', 'Resultante'])
        self.loads_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.loads_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.loads_table.setSelectionMode(QTableWidget.ExtendedSelection)
        
        # Now create the toolbar
        loads_toolbar = self.create_table_toolbar(self.loads_table, self.delete_selected_loads)
        loads_layout.addWidget(loads_toolbar)
        loads_layout.addWidget(self.loads_table)
        self.tab_widget.addTab(loads_tab, "Cargas")

        # Properties tab
        properties_tab = QWidget()
        properties_layout = QVBoxLayout(properties_tab)
        
        # Create properties table first
        self.properties_table = QTableWidget()
        self.properties_table.setColumnCount(4)
        self.properties_table.setHorizontalHeaderLabels(['Propriedade', 'Valor', 'Unidade', 'Descrição'])
        self.properties_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.properties_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.properties_table.setSelectionMode(QTableWidget.ExtendedSelection)
        
        # Now create the toolbar
        properties_toolbar = self.create_table_toolbar(self.properties_table, None)
        properties_layout.addWidget(properties_toolbar)
        properties_layout.addWidget(self.properties_table)
        self.tab_widget.addTab(properties_tab, "Propriedades")

        left_layout.addWidget(self.tab_widget)
        
        # Add node button
        add_node_btn = QPushButton('Adicionar Nó')
        add_node_btn.clicked.connect(self.add_node_mode)
        left_layout.addWidget(add_node_btn)

        # Add element button
        add_element_btn = QPushButton('Adicionar Elemento')
        add_element_btn.clicked.connect(self.add_element_mode)
        left_layout.addWidget(add_element_btn)

        # Fix node button
        fix_node_btn = QPushButton('Fixar Nó')
        fix_node_btn.clicked.connect(self.fix_node_mode)
        left_layout.addWidget(fix_node_btn)

        # Add load button
        add_load_btn = QPushButton('Adicionar Carga')
        add_load_btn.clicked.connect(self.add_load_mode)
        left_layout.addWidget(add_load_btn)

        # Material properties
        material_group = QWidget()
        material_layout = QVBoxLayout(material_group)
        
        # Distortion scale
        scale_layout = QHBoxLayout()
        scale_label = QLabel('Escala de Distorção:')
        self.scale_input = QDoubleSpinBox()
        self.scale_input.setRange(0, 1000)
        self.scale_input.setValue(100)
        self.scale_input.valueChanged.connect(self.update_distortion_scale)
        scale_layout.addWidget(scale_label)
        scale_layout.addWidget(self.scale_input)
        material_layout.addLayout(scale_layout)

        left_layout.addWidget(material_group)

        # Solve button
        solve_btn = QPushButton('Resolver')
        solve_btn.clicked.connect(self.solve)
        left_layout.addWidget(solve_btn)

        # Zoom controls
        zoom_group = QWidget()
        zoom_layout = QVBoxLayout(zoom_group)
        
        zoom_in_btn = QPushButton('Zoom +')
        zoom_in_btn.clicked.connect(self.zoom_in)
        zoom_layout.addWidget(zoom_in_btn)
        
        zoom_out_btn = QPushButton('Zoom -')
        zoom_out_btn.clicked.connect(self.zoom_out)
        zoom_layout.addWidget(zoom_out_btn)
        
        reset_zoom_btn = QPushButton('Resetar Zoom')
        reset_zoom_btn.clicked.connect(self.reset_zoom)
        zoom_layout.addWidget(reset_zoom_btn)
        
        left_layout.addWidget(zoom_group)

        left_layout.addStretch()
        layout.addWidget(left_panel)

        # Create matplotlib figure
        self.figure = Figure(figsize=(8, 6))
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.grid(True, which='both', linestyle='-', linewidth=0.5)
        self.ax.set_aspect('equal')
        self.ax.minorticks_on()
        self.ax.grid(True, which='minor', linestyle=':', linewidth=0.5)
        layout.addWidget(self.canvas)

        # Connect mouse events
        self.canvas.mpl_connect('button_press_event', self.on_click)
        self.canvas.mpl_connect('scroll_event', self.on_scroll)

        self.mode = 'none'  # Current mode: 'add_node', 'add_element', 'fix_node', 'add_load', 'none'
        self.update_plot()
        self.update_tables()

    def update_tables(self):
        # Temporarily disconnect the itemChanged signal to prevent recursive updates
        self.nodes_table.itemChanged.disconnect(self.on_node_edit)
        self.elements_table.itemChanged.disconnect(self.on_element_edit) if self.elements_table.receivers(self.elements_table.itemChanged) > 0 else None
        
        # Update nodes table
        self.nodes_table.setRowCount(len(self.truss.nodes))
        for i, (x, y) in enumerate(self.truss.nodes):
            # Node number (non-editable)
            node_item = QTableWidgetItem(str(i))
            node_item.setFlags(node_item.flags() & ~Qt.ItemIsEditable)
            node_item.setTextAlignment(Qt.AlignCenter)
            self.nodes_table.setItem(i, 0, node_item)
            
            # X coordinate (editable)
            x_item = EditableTableItem(f"{x:.2f}")
            self.nodes_table.setItem(i, 1, x_item)
            
            # Y coordinate (editable)
            y_item = EditableTableItem(f"{y:.2f}")
            self.nodes_table.setItem(i, 2, y_item)
            
            # Movement constraints (combo box)
            constraint_combo = QComboBox()
            constraint_combo.addItems(["Movable XY", "Static", "Movable X", "Movable Y"])
            
            # Set current constraint
            is_fixed_x = 2*i in self.truss.fixed_dofs
            is_fixed_y = 2*i + 1 in self.truss.fixed_dofs
            
            if is_fixed_x and is_fixed_y:
                constraint_combo.setCurrentText("Static")
            elif is_fixed_x:
                constraint_combo.setCurrentText("Movable Y")
            elif is_fixed_y:
                constraint_combo.setCurrentText("Movable X")
            else:
                constraint_combo.setCurrentText("Movable XY")
            
            # Connect the combo box signal
            constraint_combo.currentTextChanged.connect(lambda text, node=i: self.on_constraint_changed(node, text))
            self.nodes_table.setCellWidget(i, 3, constraint_combo)

        # Reconnect the itemChanged signal
        self.nodes_table.itemChanged.connect(self.on_node_edit)

        # Update elements table
        self.elements_table.setRowCount(len(self.truss.elements))
        for i, element in enumerate(self.truss.elements):
            element.calculate_properties(self.truss.nodes)
            
            # Make element number, nodes and length non-editable
            for col, value in enumerate([i, element.node1, element.node2, f"{element.L:.2f}"]):
                item = QTableWidgetItem(str(value))
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                item.setTextAlignment(Qt.AlignCenter)
                self.elements_table.setItem(i, col, item)
            
            # Make E and A editable
            E_item = QTableWidgetItem(f"{element.E:.2f}")
            E_item.setFlags(E_item.flags() | Qt.ItemIsEditable)
            E_item.setTextAlignment(Qt.AlignCenter)
            self.elements_table.setItem(i, 4, E_item)
            
            A_item = QTableWidgetItem(f"{element.A:.2f}")
            A_item.setFlags(A_item.flags() | Qt.ItemIsEditable)
            A_item.setTextAlignment(Qt.AlignCenter)
            self.elements_table.setItem(i, 5, A_item)

        # Connect the itemChanged signal for elements table
        self.elements_table.itemChanged.connect(self.on_element_edit)

        # Update loads table
        load_nodes = set()
        for dof in self.truss.loads:
            node = dof // 2
            load_nodes.add(node)
        
        self.loads_table.setRowCount(len(load_nodes))
        for i, node in enumerate(sorted(load_nodes)):
            fx = self.truss.loads.get(2*node, 0)
            fy = self.truss.loads.get(2*node+1, 0)
            resultant = np.sqrt(fx**2 + fy**2)
            
            # Make all items non-editable
            for col, value in enumerate([node, f"{fx:.2f}", f"{fy:.2f}", f"{resultant:.2f}"]):
                item = QTableWidgetItem(str(value))
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                item.setTextAlignment(Qt.AlignCenter)
                self.loads_table.setItem(i, col, item)

        # Update properties table
        self.properties_table.setRowCount(5)  # 1 for scale + 4 for chart limits
        
        # Distortion scale (editable)
        scale_item = QTableWidgetItem(str(self.scale_input.value()))
        scale_item.setFlags(scale_item.flags() | Qt.ItemIsEditable)
        scale_item.setTextAlignment(Qt.AlignCenter)
        self.properties_table.setItem(0, 0, QTableWidgetItem("Escala de Distorção"))
        self.properties_table.setItem(0, 1, scale_item)
        self.properties_table.setItem(0, 2, QTableWidgetItem(""))
        self.properties_table.setItem(0, 3, QTableWidgetItem("Fator de escala para visualização dos deslocamentos"))

        # Chart limits (editable)
        for i, (name, value, unit, description) in enumerate([
            ("Início X", self.ax.get_xlim()[0], "m", "Posição mais à esquerda do gráfico"),
            ("Início Y", self.ax.get_ylim()[0], "m", "Posição mais abaixo do gráfico"),
            ("Fim X", self.ax.get_xlim()[1], "m", "Posição mais à direita do gráfico"),
            ("Fim Y", self.ax.get_ylim()[1], "m", "Posição mais acima do gráfico")
        ], 1):
            # Make property name non-editable
            name_item = QTableWidgetItem(name)
            name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)
            name_item.setTextAlignment(Qt.AlignCenter)
            self.properties_table.setItem(i, 0, name_item)
            
            # Make value editable
            value_item = QTableWidgetItem(f"{value:.2f}")
            value_item.setFlags(value_item.flags() | Qt.ItemIsEditable)
            value_item.setTextAlignment(Qt.AlignCenter)
            self.properties_table.setItem(i, 1, value_item)
            
            # Make unit non-editable
            unit_item = QTableWidgetItem(unit)
            unit_item.setFlags(unit_item.flags() & ~Qt.ItemIsEditable)
            unit_item.setTextAlignment(Qt.AlignCenter)
            self.properties_table.setItem(i, 2, unit_item)
            
            # Make description non-editable
            desc_item = QTableWidgetItem(description)
            desc_item.setFlags(desc_item.flags() & ~Qt.ItemIsEditable)
            desc_item.setTextAlignment(Qt.AlignCenter)
            self.properties_table.setItem(i, 3, desc_item)
        
        # Connect itemChanged signal for properties table
        self.properties_table.itemChanged.connect(self.on_property_edit)

    def on_node_edit(self, item):
        try:
            # Get the row number and new value
            row = item.row()
            col = item.column()
            
            if col in [1, 2]:  # X or Y coordinate
                try:
                    new_value = float(item.text())
                except ValueError:
                    # If the input is not a valid number, revert to the old value
                    x, y = self.truss.nodes[row]
                    if col == 1:
                        item.setText(f"{x:.2f}")
                    else:
                        item.setText(f"{y:.2f}")
                    return
                
                # Update the node coordinates
                x, y = self.truss.nodes[row]
                if col == 1:  # X coordinate
                    self.truss.nodes[row] = (new_value, y)
                else:  # Y coordinate
                    self.truss.nodes[row] = (x, new_value)
                
                # Update the visualization and tables
                self.update_plot()
                self.update_tables()  # This will also format the number properly
            elif col == 3:  # Movement constraints
                new_constraint = item.text().lower()
                node = row
                
                # Remove existing constraints for this node
                if 2*node in self.truss.fixed_dofs:
                    self.truss.fixed_dofs.remove(2*node)
                if 2*node + 1 in self.truss.fixed_dofs:
                    self.truss.fixed_dofs.remove(2*node + 1)
                
                # Add new constraints based on selection
                if new_constraint == "static":
                    self.truss.fixed_dofs.add(2*node)      # Fix X
                    self.truss.fixed_dofs.add(2*node + 1)  # Fix Y
                elif new_constraint == "movable x":
                    self.truss.fixed_dofs.add(2*node + 1)  # Fix Y only
                elif new_constraint == "movable y":
                    self.truss.fixed_dofs.add(2*node)      # Fix X only
                # "Movable XY" means no constraints, so we don't add any fixed DOFs
                
                # Update the visualization
                self.update_plot()
                self.update_tables()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to update node: {str(e)}")
            self.update_tables()  # Revert to original values

    def add_node(self, x, y):
        node_id = self.truss.add_node(x, y)
        self.update_tables()
        return node_id

    def add_element(self, node1, node2):
        # Create dialog to input E and A
        dialog = QDialog(self)
        dialog.setWindowTitle('Propriedades do Elemento')
        layout = QVBoxLayout(dialog)

        # Young's modulus
        E_layout = QHBoxLayout()
        E_label = QLabel('E (MPa):')
        E_input = QDoubleSpinBox()
        E_input.setRange(0, 1000000)
        E_input.setValue(70000)  # Default value
        E_layout.addWidget(E_label)
        E_layout.addWidget(E_input)
        layout.addLayout(E_layout)

        # Cross-sectional area
        A_layout = QHBoxLayout()
        A_label = QLabel('A (mm²):')
        A_input = QDoubleSpinBox()
        A_input.setRange(0, 10000)
        A_input.setValue(100)  # Default value
        A_layout.addWidget(A_label)
        A_layout.addWidget(A_input)
        layout.addLayout(A_layout)

        # Buttons
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(dialog.accept)
        button_box.rejected.connect(dialog.reject)
        layout.addWidget(button_box)

        if dialog.exec_() == QDialog.Accepted:
            element_id = self.truss.add_element(node1, node2, E_input.value(), A_input.value())
            self.update_tables()
            return element_id
        return None

    def apply_load(self, node, fx, fy):
        self.truss.apply_load(node, fx, fy)
        self.update_tables()

    def on_click(self, event):
        if event.inaxes != self.ax:
            return

        x, y = event.xdata, event.ydata

        if self.mode == 'add_node':
            self.add_node(x, y)
            self.update_plot()
        elif self.mode == 'add_element':
            # Find closest node
            min_dist = float('inf')
            closest_node = None
            for i, (nx, ny) in enumerate(self.truss.nodes):
                dist = np.sqrt((x - nx)**2 + (y - ny)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_node = i

            if closest_node is not None:
                if len(self.selected_nodes) == 0:
                    self.selected_nodes.append(closest_node)
                else:
                    node1 = self.selected_nodes[0]
                    node2 = closest_node
                    if node1 != node2:
                        self.add_element(node1, node2)
                        self.selected_nodes = []
                        self.update_plot()
        elif self.mode == 'fix_node':
            # Find closest node
            min_dist = float('inf')
            closest_node = None
            for i, (nx, ny) in enumerate(self.truss.nodes):
                dist = np.sqrt((x - nx)**2 + (y - ny)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_node = i

            if closest_node is not None:
                self.truss.fix_node(closest_node)
                self.update_plot()
                self.update_tables()
        elif self.mode == 'add_load':
            # Find closest node
            min_dist = float('inf')
            closest_node = None
            for i, (nx, ny) in enumerate(self.truss.nodes):
                dist = np.sqrt((x - nx)**2 + (y - ny)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_node = i

            if closest_node is not None:
                self.selected_node = closest_node
                self.show_load_dialog()

    def show_load_dialog(self):
        dialog = QWidget()
        dialog.setWindowTitle('Add Load')
        layout = QVBoxLayout(dialog)

        # Horizontal load
        fx_layout = QHBoxLayout()
        fx_label = QLabel('Fx (N):')
        fx_input = QDoubleSpinBox()
        fx_input.setRange(-100000, 100000)
        fx_layout.addWidget(fx_label)
        fx_layout.addWidget(fx_input)
        layout.addLayout(fx_layout)

        # Vertical load
        fy_layout = QHBoxLayout()
        fy_label = QLabel('Fy (N):')
        fy_input = QDoubleSpinBox()
        fy_input.setRange(-100000, 100000)
        fy_layout.addWidget(fy_label)
        fy_layout.addWidget(fy_input)
        layout.addLayout(fy_layout)

        # Apply button
        apply_btn = QPushButton('Apply')
        def apply_load():
            self.apply_load(self.selected_node, fx_input.value(), fy_input.value())
            self.selected_node = None
            self.update_plot()
            dialog.close()
        apply_btn.clicked.connect(apply_load)
        layout.addWidget(apply_btn)

        dialog.setLayout(layout)
        dialog.show()

    def add_node_mode(self):
        self.mode = 'add_node'
        self.selected_nodes = []
        self.selected_node = None

    def add_element_mode(self):
        self.mode = 'add_element'
        self.selected_nodes = []
        self.selected_node = None

    def fix_node_mode(self):
        self.mode = 'fix_node'
        self.selected_nodes = []
        self.selected_node = None

    def add_load_mode(self):
        self.mode = 'add_load'
        self.selected_nodes = []
        self.selected_node = None

    def solve(self):
        try:
            displacements, forces = self.truss.solve()
            
            # Update plot with results
            self.update_plot(displacements, forces)
            
            # Show results in a message box
            msg = QMessageBox()
            msg.setWindowTitle("Resultados da Análise")
            msg.setText("Análise concluída com sucesso!")
            msg.setInformativeText(f"Deslocamento máximo: {np.max(np.abs(displacements)):.4f} mm\n"
                                 f"Força máxima no elemento: {np.max(np.abs(forces)):.2f} N")
            msg.exec_()
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Falha na análise: {str(e)}")

    def update_plot(self, displacements=None, forces=None):
        self.ax.clear()
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.grid(True, which='both', linestyle='-', linewidth=0.5)
        self.ax.set_aspect('equal')
        self.ax.minorticks_on()
        self.ax.grid(True, which='minor', linestyle=':', linewidth=0.5)

        # Plot nodes
        for i, (x, y) in enumerate(self.truss.nodes):
            self.ax.plot(x, y, 'ko')
            self.ax.text(x, y, str(i), fontsize=12)

        # Plot elements
        for i, element in enumerate(self.truss.elements):
            x1, y1 = self.truss.nodes[element.node1]
            x2, y2 = self.truss.nodes[element.node2]
            
            if forces is not None:
                # Color elements based on force
                force = forces[i]
                if force > 0:
                    color = 'r'  # Tension
                else:
                    color = 'b'  # Compression
                self.ax.plot([x1, x2], [y1, y2], color=color, linewidth=2)
            else:
                self.ax.plot([x1, x2], [y1, y2], 'k-')

        # Plot fixed nodes
        for i, (x, y) in enumerate(self.truss.nodes):
            if 2*i in self.truss.fixed_dofs and 2*i+1 in self.truss.fixed_dofs:
                self.ax.plot(x, y, 'g^', markersize=10)
            elif 2*i in self.truss.fixed_dofs:
                self.ax.plot(x, y, 'g>', markersize=10)
            elif 2*i+1 in self.truss.fixed_dofs:
                self.ax.plot(x, y, 'g^', markersize=10)

        # Plot loads
        for dof, force in self.truss.loads.items():
            node = dof // 2
            x, y = self.truss.nodes[node]
            if dof % 2 == 0:  # Horizontal load
                self.ax.arrow(x, y, force/1000, 0, head_width=0.2, head_length=0.3, fc='r', ec='r')
            else:  # Vertical load
                self.ax.arrow(x, y, 0, force/1000, head_width=0.2, head_length=0.3, fc='r', ec='r')

        # Plot deformed shape if displacements are available
        if displacements is not None:
            scale = self.distortion_scale  # Use the distortion scale property
            for i, element in enumerate(self.truss.elements):
                x1, y1 = self.truss.nodes[element.node1]
                x2, y2 = self.truss.nodes[element.node2]
                ux1, uy1 = displacements[2*element.node1], displacements[2*element.node1+1]
                ux2, uy2 = displacements[2*element.node2], displacements[2*element.node2+1]
                self.ax.plot([x1 + ux1*scale, x2 + ux2*scale], 
                           [y1 + uy1*scale, y2 + uy2*scale], 
                           'r--', alpha=0.5)

        self.canvas.draw()

    def on_scroll(self, event):
        if event.inaxes != self.ax:
            return

        # Get the current x and y limits
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()

        # Get the current mouse position
        x = event.xdata
        y = event.ydata

        # Calculate the zoom factor (more gradual zoom)
        if event.button == 'up':
            # Zoom in
            zoom_factor = 1.1
        else:
            # Zoom out
            zoom_factor = 0.9

        # Calculate new limits
        x_range = (xlim[1] - xlim[0]) * zoom_factor
        y_range = (ylim[1] - ylim[0]) * zoom_factor

        # Calculate new limits while keeping the mouse position fixed
        x_center = x
        y_center = y

        # Set new limits
        self.ax.set_xlim(x_center - x_range/2, x_center + x_range/2)
        self.ax.set_ylim(y_center - y_range/2, y_center + y_range/2)

        # Update zoom level
        if event.button == 'up':
            self.zoom_level *= zoom_factor
        else:
            self.zoom_level /= zoom_factor

        # Update the plot
        self.canvas.draw()

    def zoom_in(self):
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        x_center = (xlim[0] + xlim[1]) / 2
        y_center = (ylim[0] + ylim[1]) / 2
        x_range = (xlim[1] - xlim[0]) / 1.1
        y_range = (ylim[1] - ylim[0]) / 1.1
        self.ax.set_xlim(x_center - x_range/2, x_center + x_range/2)
        self.ax.set_ylim(y_center - y_range/2, y_center + y_range/2)
        self.zoom_level *= 1.1
        self.canvas.draw()

    def zoom_out(self):
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        x_center = (xlim[0] + xlim[1]) / 2
        y_center = (ylim[0] + ylim[1]) / 2
        x_range = (xlim[1] - xlim[0]) * 1.1
        y_range = (ylim[1] - ylim[0]) * 1.1
        self.ax.set_xlim(x_center - x_range/2, y_center + y_range/2)
        self.ax.set_ylim(y_center - y_range/2, y_center + y_range/2)
        self.zoom_level /= 1.1
        self.canvas.draw()

    def reset_zoom(self):
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.zoom_level = 1.0
        self.canvas.draw()

    def delete_selected_nodes(self):
        selected_rows = sorted([item.row() for item in self.nodes_table.selectedItems()])
        if not selected_rows:
            return
            
        # Remove duplicates and sort in reverse order to avoid index shifting
        selected_rows = sorted(list(set(selected_rows)), reverse=True)
        
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setText("Delete Selected Nodes?")
        msg.setInformativeText("This will also delete all connected elements and loads.")
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        
        if msg.exec_() == QMessageBox.Yes:
            for row in selected_rows:
                # Remove all elements connected to this node
                elements_to_remove = []
                for i, element in enumerate(self.truss.elements):
                    if element.node1 == row or element.node2 == row:
                        elements_to_remove.append(i)
                
                # Remove elements in reverse order
                for element_idx in sorted(elements_to_remove, reverse=True):
                    self.truss.elements.pop(element_idx)
                
                # Remove loads on this node
                dofs_to_remove = [2*row, 2*row + 1]
                for dof in dofs_to_remove:
                    if dof in self.truss.loads:
                        del self.truss.loads[dof]
                
                # Remove the node
                self.truss.nodes.pop(row)
                
                # Update node indices in elements
                for element in self.truss.elements:
                    if element.node1 > row:
                        element.node1 -= 1
                    if element.node2 > row:
                        element.node2 -= 1
                
                # Update node indices in loads
                new_loads = {}
                for dof, force in self.truss.loads.items():
                    node = dof // 2
                    if node > row:
                        new_dof = dof - 2  # Shift by 2 because each node has 2 DOFs
                        new_loads[new_dof] = force
                    elif node < row:
                        new_loads[dof] = force
                self.truss.loads = new_loads

            self.update_plot()
            self.update_tables()

    def delete_selected_elements(self):
        selected_rows = sorted([item.row() for item in self.elements_table.selectedItems()])
        if not selected_rows:
            return
            
        # Remove duplicates and sort in reverse order to avoid index shifting
        selected_rows = sorted(list(set(selected_rows)), reverse=True)
        
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setText("Delete Selected Elements?")
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        
        if msg.exec_() == QMessageBox.Yes:
            for row in selected_rows:
                self.truss.elements.pop(row)
            
            self.update_plot()
            self.update_tables()

    def delete_selected_loads(self):
        selected_rows = sorted([item.row() for item in self.loads_table.selectedItems()])
        if not selected_rows:
            return
            
        # Remove duplicates and sort in reverse order
        selected_rows = sorted(list(set(selected_rows)), reverse=True)
        
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setText("Delete Selected Loads?")
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        
        if msg.exec_() == QMessageBox.Yes:
            for row in selected_rows:
                node = int(self.loads_table.item(row, 0).text())
                # Remove both Fx and Fy for the node
                if 2*node in self.truss.loads:
                    del self.truss.loads[2*node]
                if 2*node + 1 in self.truss.loads:
                    del self.truss.loads[2*node + 1]
            
            self.update_plot()
            self.update_tables()

    def save_structure(self):
        filename, _ = QFileDialog.getSaveFileName(self, 'Salvar Estrutura', '', 'Arquivos JSON (*.json);;Todos os Arquivos (*)')
        if filename:
            try:
                if not filename.endswith('.json'):
                    filename += '.json'
                self.truss.save_to_file(filename)
                QMessageBox.information(self, "Sucesso", "Estrutura salva com sucesso!")
            except Exception as e:
                QMessageBox.critical(self, "Erro", f"Falha ao salvar a estrutura: {str(e)}")

    def load_structure(self):
        filename, _ = QFileDialog.getOpenFileName(self, 'Carregar Estrutura', '', 'Arquivos JSON (*.json);;Todos os Arquivos (*)')
        if filename:
            try:
                self.truss.load_from_file(filename)
                self.update_plot()
                self.update_tables()
                QMessageBox.information(self, "Sucesso", "Estrutura carregada com sucesso!")
            except Exception as e:
                QMessageBox.critical(self, "Erro", f"Falha ao carregar a estrutura: {str(e)}")

    def get_element_info(self):
        """Get information about each element"""
        info = []
        for i, element in enumerate(self.truss.elements):
            force = self.truss.forces[i] if self.truss.forces is not None else 0
            force_type = "Tração" if force > 0 else "Compressão"
            info.append({
                'element': i,
                'force': force,
                'force_type': force_type,
                'L': element.L,
                'cos': element.cos,
                'sin': element.sin
            })
        return info

    def on_property_edit(self, item):
        """Manipula a edição de propriedades"""
        try:
            row = item.row()
            col = item.column()

            if col != 1:
                return

            try:
                new_value = float(item.text())
            except ValueError:
                raise ValueError("Por favor, insira um número válido")

            if row == 0:  # Distortion scale
                if new_value <= 0:
                    raise ValueError("A escala deve ser positiva")
                self.scale_input.setValue(new_value)
                if hasattr(self.truss, 'displacements') and self.truss.displacements is not None:
                    self.update_plot(self.truss.displacements, self.truss.forces)
            elif row >= 1 and row <= 4:  # Chart limits
                xlim = list(self.ax.get_xlim())
                ylim = list(self.ax.get_ylim())
                
                if row == 1:  # Início X
                    if new_value >= xlim[1]:
                        raise ValueError("O início X deve ser menor que o fim X")
                    xlim[0] = new_value
                elif row == 2:  # Início Y
                    if new_value >= ylim[1]:
                        raise ValueError("O início Y deve ser menor que o fim Y")
                    ylim[0] = new_value
                elif row == 3:  # Fim X
                    if new_value <= xlim[0]:
                        raise ValueError("O fim X deve ser maior que o início X")
                    xlim[1] = new_value
                elif row == 4:  # Fim Y
                    if new_value <= ylim[0]:
                        raise ValueError("O fim Y deve ser maior que o início Y")
                    ylim[1] = new_value
                
                self.ax.set_xlim(xlim)
                self.ax.set_ylim(ylim)
                self.canvas.draw()

            item.setText(f"{new_value:.2f}")
            item.setTextAlignment(Qt.AlignCenter)

        except Exception as e:
            QMessageBox.critical(self, "Erro", str(e))
            self.update_tables()

    def update_distortion_scale(self):
        """Update the distortion scale for all elements"""
        scale = self.scale_input.value()
        
        # Update all existing elements
        for element in self.truss.elements:
            element.distortion_scale = scale
        
        # If we have results, recalculate
        if hasattr(self.truss, 'displacements') and self.truss.displacements is not None:
            try:
                displacements, forces = self.truss.solve()
                self.update_plot(displacements, forces)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to update analysis: {str(e)}")

    def on_constraint_changed(self, node, text):
        # Remove existing constraints for this node
        if 2*node in self.truss.fixed_dofs:
            self.truss.fixed_dofs.remove(2*node)
        if 2*node + 1 in self.truss.fixed_dofs:
            self.truss.fixed_dofs.remove(2*node + 1)
        
        # Add new constraints based on selection
        if text == "Static":
            self.truss.fixed_dofs.add(2*node)      # Fix X
            self.truss.fixed_dofs.add(2*node + 1)  # Fix Y
        elif text == "Movable X":
            self.truss.fixed_dofs.add(2*node + 1)  # Fix Y only
        elif text == "Movable Y":
            self.truss.fixed_dofs.add(2*node)      # Fix X only
        # "Movable XY" means no constraints, so we don't add any fixed DOFs
        
        # Update the visualization
        self.update_plot()
        self.update_tables()

    def on_element_edit(self, item):
        """Manipula a edição das propriedades dos elementos"""
        try:
            row = item.row()
            col = item.column()

            if col not in [4, 5]:  # Apenas E e A são editáveis
                return

            try:
                new_value = float(item.text())
            except ValueError:
                raise ValueError("Por favor, insira um número válido")

            if new_value <= 0:
                raise ValueError("O valor deve ser positivo")

            # Atualiza o valor no elemento
            element = self.truss.elements[row]
            if col == 4:  # E
                element.E = new_value
            else:  # A
                element.A = new_value

            # Se temos resultados, recalcula
            if hasattr(self.truss, 'displacements') and self.truss.displacements is not None:
                try:
                    displacements, forces = self.truss.solve()
                    self.update_plot(displacements, forces)
                except Exception as e:
                    QMessageBox.critical(self, "Erro", f"Falha ao atualizar análise: {str(e)}")

            # Formata o número
            item.setText(f"{new_value:.2f}")
            item.setTextAlignment(Qt.AlignCenter)

        except Exception as e:
            QMessageBox.critical(self, "Erro", str(e))
            self.update_tables()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = TrussGUI()
    ex.show()
    sys.exit(app.exec_()) 