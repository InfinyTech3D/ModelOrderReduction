import Sofa
import Sofa.Core
import os

class MOController(Sofa.Core.Controller):
    def __init__(self, state):
         ## These are needed (and the normal way to override from a python class)
         #Sofa.Core.Controller.__init__(self, *args, **kwargs)
         super().__init__()
         #self.mechanical_object = kwargs.get("mechanical_object")
         self.state = state
         

    def onEvent(self, event):
         """This function is the fallback one that is called if the XXXX event is
            received but there is not overriden onXXXX() method.
         """
         print("generic event handler catched ", event)

    def onAnimateBeginEvent(self, event):
         print("onAnimateBeginEvent")

         # Access the position of the particle 
         particles_mecha = self.state.position.value

         print('====================================')
         print('State of the particle')
         print('====================================')
         print('state vector: '+str(particles_mecha))

         # Change the the value of the position in z direction
         with self.state.position.writeable() as state:
             for i in range(len(state)): 
                  state[i][2] += 1
                  print('state vector after modification: '+str(state))
                     


meshPath = os.path.dirname(os.path.abspath(__file__))+'/mesh/'
plugins=["SofaPython3","SoftRobots","ModelOrderReduction","STLIB",
        
         "Sofa.Component.Visual",
         "Sofa.Component.AnimationLoop",
         "Sofa.GL.Component.Rendering3D",
         "Sofa.Component.Constraint.Lagrangian.Solver",
         'Sofa.Component.Collision.Detection.Algorithm',
         'Sofa.Component.Collision.Detection.Intersection',
         'Sofa.Component.Collision.Geometry',
         'Sofa.Component.Collision.Response.Contact',
         'Sofa.Component.IO.Mesh',
         'Sofa.Component.Playback',
         'Sofa.Component.Constraint.Lagrangian.Correction', # Needed to use components [GenericConstraintCorrection]
         'Sofa.Component.Engine.Select', # Needed to use components [BoxROI]
         'Sofa.Component.LinearSolver.Direct', # Needed to use components [SparseLDLSolver]
         'Sofa.Component.LinearSolver.Iterative', # Needed to use components [CGLinearSolver] 
         'Sofa.Component.Mapping.Linear', # Needed to use components [BarycentricMapping]
         'Sofa.Component.Mass', # Needed to use components [UniformMass]
         'Sofa.Component.ODESolver.Backward', # Needed to use components [EulerImplicitSolver]
         'Sofa.Component.SolidMechanics.FEM.Elastic', # Needed to use components [TetrahedronFEMForceField]
         'Sofa.Component.SolidMechanics.Spring', # Needed to use components [RestShapeSpringsForceField]
         'Sofa.Component.StateContainer', # Needed to use components [MechanicalObject]
         'Sofa.Component.Topology.Container.Dynamic', # Needed to use components [TetrahedronSetTopologyContainer]
         'Sofa.Component.Constraint.Projective', # Needed to use components [FixedProjectiveConstraint]
         'Sofa.Component.Topology.Container.Grid'] # Needed to use components [RegularGridTopology]

def createScene(rootNode):
   
    
    rootNode.addObject('RequiredPlugin', pluginName=plugins, printLog=False)

    # Animation loop
    rootNode.addObject('DefaultAnimationLoop')

    # Visual manager loop
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('VisualStyle', displayFlags='showCollisionModels showVisualModels showForceFields showInteractionForceFields hideWireframe')

    # Collision pipeline
    rootNode.addObject('CollisionPipeline', verbose="0")
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response='PenalityContactForceField')
    rootNode.addObject('MinProximityIntersection', name='Proximity', alarmDistance='1.0', contactDistance='0.1')
    
    # Add time step and the gravity force
    rootNode.findData('dt').value=0.03
    rootNode.findData('gravity').value=[0, 0, 0]

    # Add time integration scheme and solver
    rootNode.addObject('EulerImplicitSolver', name='EulerImplicit',  rayleighStiffness='0.1', rayleighMass='0.1')
    rootNode.addObject('CGLinearSolver', name='CG Solver', iterations='25', tolerance='1e-9', threshold='1e-9')
    # Add carving manager

    # Add carving element
    carvingElement = rootNode.addChild('carvingElement')

    # Access the mechanical object 
    particles = carvingElement.addObject('MechanicalObject', name='Particles', template='Vec3d', position='5 5 1', velocity='0 0 0')
    #carvingElement.addObject(MOController('MechanicalObject', name= 'StateAccessor', mechanical_object=particles))
    carvingElement.addObject(MOController(state=particles))

    carvingElement.addObject('UniformMass', name='Mass', totalMass='10.0')
    carvingElement.addObject('CGLinearSolver', iterations="200", tolerance="1e-09", threshold="1e-09")
    carvingElement.addObject('ConstantForceField', totalForce='-1 0 0 0 0 0')
    carvingElement.addObject('SphereCollisionModel', name='tool', radius="0.1", tags="CarvingTool")

    #Add the liver node
    liver = rootNode.addChild('liver')
    liver.addObject('EulerImplicitSolver', rayleighStiffness = 0.0, rayleighMass = 0.0)
    liver.addObject('SparseLDLSolver',template="CompressedRowSparseMatrixMat3x3d")
    liver.addObject('MeshVTKLoader', name="loader", filename=meshPath+'liverFine.vtu')
    liver.addObject('TetrahedronSetTopologyContainer', name='liver_topo' , src="@loader")
    liver.addObject('MechanicalObject', name="MO")
    liver.addObject('BoxROI', name='ROI1', box='0 3 -1 2 5 2', drawBoxes='true')
    liver.addObject('BoxROI', name='boxROIactuation', box='-5 0 -0.5 -4 0.5 0.5', drawBoxes='true')

    liver.addObject('UniformMass', totalMass=0.3)
    liver.addObject('TetrahedronFEMForceField', poissonRatio="0.3", youngModulus="2000")
    liver.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness = '1e8')
    
    # Add a visual model
    visu = liver.addChild('visu')
    visu.addObject( 'MeshOBJLoader', name= 'loader', filename=meshPath+'liver-smoothUV.obj')
    visu.addObject('OglModel',src='@loader')
    visu.addObject('BarycentricMapping')

    # Add a collision model
    collision= liver.addChild('collision')
    collision.addObject('TriangleSetTopologyContainer', name='container')
    collision.addObject('TriangleSetTopologyModifier', name='modifier')
    collision.addObject('TriangleSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d', drawTriangles='0')
    collision.addObject('Tetra2TriangleTopologicalMapping', input='@../liver_topo', output='@container')
    collision.addObject('TriangleCollisionModel', name='triangleCol')
    collision.addObject('PointCollisionModel', name='pointCol')

    # Add an actuator
    actuator = rootNode.addChild('actuator')
    actuator.addObject('MechanicalObject', name = 'actuatorState', position = '@liver/MO.position', template = 'Vec3d')
