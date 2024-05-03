# -*- coding: utf-8 -*-
import os
import Sofa
from numpy import add,subtract,multiply
try:
    from splib3.numerics import *
except:
    raise ImportError("ModelOrderReduction plugin depend on SPLIB"\
                     +"Please install it : https://github.com/SofaDefrost/STLIB")

path = os.path.dirname(os.path.abspath(__file__))

def TRSinOrigin(positions,modelPosition,translation,rotation,scale=[1.0,1.0,1.0]):
    posOrigin = subtract(positions , modelPosition)
    if any(isinstance(el, list) for el in positions):
        posOriginTRS = transformPositions(posOrigin,translation,eulerRotation=rotation,scale=scale)
    else:
        posOriginTRS = transformPosition(posOrigin,TRS_to_matrix(translation,eulerRotation=rotation,scale=scale))
    return add(posOriginTRS,modelPosition).tolist()
    
def newBox(positions,modelPosition,translation,rotation,offset,scale=[1.0,1.0,1.0]):
    pos = TRSinOrigin(positions,modelPosition,translation,rotation,scale)
    offset =transformPositions([offset],eulerRotation=rotation,scale=scale)[0]
    return add(pos,offset).tolist()

def Reduced_test(
                  attachedTo=None,
                  name="Reduced_test",
                  rotation=[0.0, 0.0, 0.0],
                  translation=[0.0, 0.0, 0.0],
                  scale=[1.0, 1.0, 1.0],
                  surfaceMeshFileName=False,
                  surfaceColor=[1.0, 1.0, 1.0],
                  nbrOfModes=18,
                  hyperReduction=True):
    """
    Object with an elastic deformation law.

        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | argument            | type      | definition                                                                                      |
        +=====================+===========+=================================================================================================+
        | attachedTo          | Sofa.Node | Where the node is created;                                                                      |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | name                | str       | name of the Sofa.Node it will                                                                   |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | rotation            | vec3f     | Apply a 3D rotation to the object in Euler angles.                                              |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | translation         | vec3f     | Apply a 3D translation to the object.                                                           |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | scale               | vec3f     | Apply a 3D scale to the object.                                                                 |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | surfaceMeshFileName | str       | Filepath to a surface mesh (STL, OBJ). If missing there is no visual properties to this object. |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | surfaceColor        | vec3f     | The default color used for the rendering of the object.                                         |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | nbrOfModes          | int       | Number of modes we want our reduced model to work with                                          |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | hyperReduction      | Bool      | Controlled if we have the simple reduction or the hyper-reduction                               |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+

    """

    modelRoot = attachedTo.addChild(name)

    liver_MOR = modelRoot.addChild('liver_MOR')
    liver_MOR.addObject('EulerImplicitSolver' , rayleighStiffness = 0.0, rayleighMass = 0.0)
    liver_MOR.addObject('SparseLDLSolver' , name = 'solver', template = 'CompressedRowSparseMatrixMat3x3d')
    liver_MOR.addObject('GenericConstraintCorrection')
    liver_MOR.addObject('MechanicalObject' , template = 'Vec1d', position = [0]*nbrOfModes)


    liver = liver_MOR.addChild('liver')
    liver.addObject('MeshVTKLoader' , name = 'loader', filename = path + r'\mesh\liverFine.vtu', translation = add(translation,[0.0, 0.0, 0.0]), rotation = add(rotation,[0.0, 0.0, 0.0]), scale3d = multiply(scale,[1.0, 1.0, 1.0]))
    liver.addObject('TetrahedronSetTopologyContainer' , name = 'liver_topo', src = '@loader')
    liver.addObject('MechanicalObject' , name = 'MO')
    liver.addObject('BoxROI' , name= 'ROI1' , orientedBox= newBox([[0.0, 5.0, -1.0], [0.0, 3.0, -1.0], [2.0, 3.0, -1.0]] , [0.0, 0.0, 0.0],translation,rotation,[0, 0, 1.5],scale) + multiply(scale[2],[3.0]).tolist(),drawBoxes=True)
    liver.addObject('BoxROI' , name= 'boxROIactuation' , orientedBox= newBox([[-5.0, 0.5, -0.5], [-5.0, 0.0, -0.5], [-4.0, 0.0, -0.5]] , [0.0, 0.0, 0.0],translation,rotation,[0, 0, 0.5],scale) + multiply(scale[2],[1.0]).tolist(),drawBoxes=True)
    liver.addObject('UniformMass' , totalMass = 0.3)
    liver.addObject('HyperReducedTetrahedronFEMForceField' , poissonRatio = '0.3', youngModulus = '500', name = 'reducedFF_liver_0', nbModes = nbrOfModes, performECSW = hyperReduction, modesPath = path + r'\data\modes.txt', RIDPath = path + r'\data\reducedFF_liver_0_RID.txt', weightsPath = path + r'\data\reducedFF_liver_0_weight.txt')
    liver.addObject('HyperReducedRestShapeSpringsForceField' , points = '@ROI1.indices', stiffness = '1e8', name = 'reducedFF_liver_1', nbModes = nbrOfModes, performECSW = hyperReduction, modesPath = path + r'\data\modes.txt', RIDPath = path + r'\data\reducedFF_liver_1_RID.txt', weightsPath = path + r'\data\reducedFF_liver_1_weight.txt')
    liver.addObject('ModelOrderReductionMapping' , input = '@../MechanicalObject', modesPath = path + r'\data\modes.txt', output = '@./MO')


    visu = liver.addChild('visu')
    visu.addObject('MeshOBJLoader' , name = 'loader', filename = path + r'\mesh\liver-smoothUV.obj', translation = add(translation,[0.0, 0.0, 0.0]), rotation = add(rotation,[0.0, 0.0, 0.0]), scale3d = multiply(scale,[1.0, 1.0, 1.0]))
    visu.addObject('OglModel' , src = '@loader')
    visu.addObject('BarycentricMapping')


    collision = liver.addChild('collision')
    collision.addObject('TriangleSetTopologyContainer' , name = 'container')
    collision.addObject('TriangleSetTopologyModifier' , name = 'modifier')
    collision.addObject('TriangleSetGeometryAlgorithms' , name = 'GeomAlgo', template = 'Vec3d', drawTriangles = '0')
    collision.addObject('Tetra2TriangleTopologicalMapping' , input = '@../liver_topo', output = '@container')
    collision.addObject('TriangleCollisionModel' , name = 'triangleCol')
    collision.addObject('PointCollisionModel' , name = 'pointCol')


    actuatorLiver = modelRoot.addChild('actuatorLiver')
    actuatorLiver.addObject('MechanicalObject' , name = 'actuatorLiver', position = '@liver/MO.position', template = 'Vec3d')

    return liver


#   STLIB IMPORT
from stlib3.scene import MainHeader
def createScene(rootNode):
    surfaceMeshFileName = False

    MainHeader(rootNode,plugins=["SoftRobots","ModelOrderReduction"],
                        dt=0.03,
                        gravity=[0.0, 0.0, 0.0])
    rootNode.VisualStyle.displayFlags="showForceFields"

    rootNode.addObject('FreeMotionAnimationLoop')
    # Collision pipeline
    rootNode.addObject('CollisionPipeline', verbose="0")
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response='FrictionContactConstraint') #Used with FreeMotionAnimationLoop
    rootNode.addObject('GenericConstraintSolver')
    rootNode.addObject('MinProximityIntersection', name='Proximity', alarmDistance='1.0', contactDistance='0.1')
    
    # Add carving element ------------------------------------------------------------------------------------------------

    carvingElement = rootNode.addChild('carvingElement')

    # Access the mechanical object 
    particles = carvingElement.addObject('MechanicalObject', name='Particles', template='Vec3d', position='-2 3 3.5', velocity='0 0 0')
    
    carvingElement.addObject('UniformMass', name='Mass', totalMass='40.0')
    carvingElement.addObject('EulerImplicitSolver', name='EulerImplicit',  rayleighStiffness='0.1', rayleighMass='0.1')
    carvingElement.addObject('CGLinearSolver', name="solver" ,iterations="200", tolerance="1e-09", threshold="1e-09")
    carvingElement.addObject('ConstantForceField', totalForce='0 0 -10 0 0 0')
    carvingElement.addObject('SphereCollisionModel', name='tool', radius="0.1", tags="CarvingTool")
    carvingElement.addObject('UncoupledConstraintCorrection') # Related to FreeMotionAnimationLoop
    
    #-----------------------------------------------------------------------------------------------


    Reduced_test(rootNode,
                        name="Reduced_test",
                        surfaceMeshFileName=surfaceMeshFileName)

    # translate = 300
    # rotationBlue = 60.0
    # rotationWhite = 80
    # rotationRed = 70

    # for i in range(3):

    #     Reduced_test(rootNode,
    #                    name="Reduced_test_blue_"+str(i),
    #                    rotation=[rotationBlue*i, 0.0, 0.0],
    #                    translation=[i*translate, 0.0, 0.0],
    #                    surfaceColor=[0.0, 0.0, 1, 0.5],
    #                    surfaceMeshFileName=surfaceMeshFileName)
    # for i in range(3):

    #     Reduced_test(rootNode,
    #                    name="Reduced_test_white_"+str(i),
    #                    rotation=[0.0, rotationWhite*i, 0.0],
    #                    translation=[i*translate, translate, -translate],
    #                    surfaceColor=[0.5, 0.5, 0.5, 0.5],
    #                    surfaceMeshFileName=surfaceMeshFileName)

    # for i in range(3):

    #     Reduced_test(rootNode,
    #                    name="Reduced_test_red_"+str(i),
    #                    rotation=[0.0, 0.0, i*rotationRed],
    #                    translation=[i*translate, 2*translate, -2*translate],
    #                    surfaceColor=[1, 0.0, 0.0, 0.5],
    #                    surfaceMeshFileName=surfaceMeshFileName)
