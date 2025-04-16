# Truss Analysis Program

This program provides a graphical interface for analyzing 2D truss structures using the Finite Element Method (FEM). It allows you to create truss structures, apply loads, and visualize the results including displacements and internal forces.

## Features

- Interactive graphical interface for creating truss structures
- Add nodes and connect them with elements
- Apply fixed supports to nodes
- Apply point loads in both horizontal and vertical directions
- Visualize deformed shape and internal forces
- Calculate displacements and axial forces in elements

## Installation

1. Make sure you have Python 3.7 or later installed
2. Install the required dependencies:

```bash
pip install -r requirements.txt
```

## Usage

1. Run the program:

```bash
python truss_gui.py
```

2. Using the interface:

   - Click "Add Node" and click on the canvas to place nodes
   - Click "Add Element" and select two nodes to connect them
   - Click "Fix Node" and select a node to add supports
   - Click "Add Load" and select a node to apply forces
   - Set material properties (Young's modulus and cross-sectional area)
   - Click "Solve" to analyze the structure

3. Results:
   - The program will show the deformed shape (dashed red lines)
   - Elements in tension are shown in red
   - Elements in compression are shown in blue
   - Fixed supports are shown in green
   - Applied loads are shown as red arrows

## Example

1. Create a simple triangular truss:
   - Add three nodes to form a triangle
   - Connect the nodes with elements
   - Fix the bottom left node
   - Apply a vertical load at the top node
   - Click "Solve" to see the results

## Notes

- The program uses SI units (N, mm, MPa)
- Default material properties are set for steel (E = 210000 MPa)
- The displacement scale factor is set to 10 for better visualization
- The program will show an error message if the structure is unstable or if there are any issues with the analysis
# mec
