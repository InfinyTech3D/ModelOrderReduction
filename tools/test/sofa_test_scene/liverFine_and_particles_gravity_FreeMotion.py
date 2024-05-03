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

    def onAnimateEndEvent(self, event):
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
                  #state[i][2] += 0.01
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
    rootNode.addObject('FreeMotionAnimationLoop')

    # Visual manager loop
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('VisualStyle', displayFlags='showCollisionModels showForceFields hideWireframe')

    # Collision pipeline
    rootNode.addObject('CollisionPipeline', verbose="0")
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response='FrictionContactConstraint') #Used with FreeMotionAnimationLoop
    rootNode.addObject('GenericConstraintSolver')
    rootNode.addObject('MinProximityIntersection', name='Proximity', alarmDistance='1.0', contactDistance='0.1')
    
    # Add time step and the gravity force
    rootNode.findData('dt').value=0.03
    rootNode.findData('gravity').value=[0, -5, 0]

   # Add carving element------------------------------------------------------------------------------------------------

    carvingElement = rootNode.addChild('carvingElement')

    # Access the mechanical object 
    particles = carvingElement.addObject('MechanicalObject', name='Particles', template='Vec3d', position='-2 7 1', velocity='0 0 0')
    carvingElement.addObject(MOController(state=particles))

    carvingElement.addObject('UniformMass', name='Mass', totalMass='40.0')
    carvingElement.addObject('EulerImplicitSolver', name='EulerImplicit',  rayleighStiffness='0.1', rayleighMass='0.1')
    carvingElement.addObject('CGLinearSolver', name="solver" ,iterations="200", tolerance="1e-09", threshold="1e-09")
    carvingElement.addObject('SphereCollisionModel', name='tool', radius="0.1", tags="CarvingTool")
    carvingElement.addObject('UncoupledConstraintCorrection') # Related to FreeMotionAnimationLoop
    
    # Add the liver node-----------------------------------------------------------------------------------------------
    
    liver = rootNode.addChild('liver')

    liver.addObject('EulerImplicitSolver', rayleighStiffness = 0.0, rayleighMass = 0.0)
    liver.addObject('SparseLDLSolver', name="solver" ,template="CompressedRowSparseMatrixMat3x3d")
    
    liver.addObject('GenericConstraintCorrection') # Related to FreeMotionAnimationLoop
    liver.addObject('MeshVTKLoader', name="loader", filename=meshPath+'liverFine.vtu')
    liver.addObject('TetrahedronSetTopologyContainer', name='liver_topo' , src="@loader")
    liver.addObject('MechanicalObject', name="MO")
    liver.addObject('BoxROI', name='ROI1', box='0 3 -1 2 5 2', drawBoxes='true')
    liver.addObject('BoxROI', name='boxROIactuation', box='-5 0 -0.5 -4 0.5 0.5', drawBoxes='true')

    liver.addObject('UniformMass', totalMass=0.3)
    liver.addObject('TetrahedronFEMForceField', poissonRatio="0.3", youngModulus="500")
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
   
    #  Add an actuator for liver-----------------------------------------------------------------------------------------------

    actuatorLiver = rootNode.addChild('actuatorLiver')
    actuatorLiver.addObject('MechanicalObject', name = 'actuatorLiver', position = '@liver/MO.position', template = 'Vec3d')
  
    # Add an actuator for particles---------------------------------------------------------------------------------------------
    #actuatorParticles = rootNode.addChild('actuatorParticles')
    #actuatorParticles.addObject('MechanicalObject', name = 'actuatorParticles', position = '@carvingElement/Particles.position', template = 'Vec3d')
