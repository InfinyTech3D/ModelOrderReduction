import Sofa
import Sofa.Core
import os

meshPath = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

def createScene(rootNode):
    
    rootNode.addObject('RequiredPlugin', name='ModelOrderReduction', pluginName='ModelOrderReduction')
    rootNode.addObject('RequiredPlugin', name='SofaPython3', pluginName='SofaPython3')

    rootNode.addObject('VisualStyle', displayFlags='showCollision showVisualModels showForceFields showInteractionForceFields hideCollisionModels hideBoundingCollisionModels hideWireframe')
    rootNode.findData('dt').value=0.01
    rootNode.findData('gravity').value=[0, -981, 0]
    surfaceColor=[0.7, 0.7, 0.7, 0.7]

    liver = rootNode.addChild('liver')
    liver.addObject('EulerImplicitSolver', rayleighStiffness = 0.0, rayleighMass = 0.0)
    liver.addObject('SparseLDLSolver',template="CompressedRowSparseMatrixMat3x3d")
    liver.addObject('MeshVTKLoader', name="loader", filename=meshPath+'liverFine.vtu')
    liver.addObject('TetrahedronSetTopologyContainer', src="@loader")
    liver.addObject('MechanicalObject', name="MO")
    liver.addObject('BoxROI', name='ROI1', box='0 3 -1 2 5 2', drawBoxes='true')
    liver.addObject('BoxROI', name='boxROIactuation', box='-5 0 -0.5 -4 0.5 0.5', drawBoxes='true')

    liver.addObject('UniformMass', totalMass=0.3)
    liver.addObject('TetrahedronFEMForceField', poissonRatio="0.3", youngModulus="5000")
    liver.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness = '1e8')
    
    # Add a visual model
    visu = liver.addChild('visu')
    visu.addObject(  'MeshOBJLoader', name= 'loader', filename=meshPath+'liver-smoothUV.obj')
    visu.addObject('OglModel',src='@loader',  color=list(surfaceColor))
    visu.addObject('BarycentricMapping')

    # Add an actuator
    actuator = rootNode.addChild('actuator')
    actuator.addObject('MechanicalObject', name = 'actuatorState', position = '@liver/MO.position', template = 'Vec3d')
