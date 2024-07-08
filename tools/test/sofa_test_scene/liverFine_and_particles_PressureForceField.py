import Sofa
import Sofa.Core
import os

class MOController(Sofa.Core.Controller):
    def __init__(self, state, pressure):
         ## These are needed (and the normal way to override from a python class)
         #Sofa.Core.Controller.__init__(self, *args, **kwargs)
         super().__init__()
         #self.mechanical_object = kwargs.get("mechanical_object")
         self.state = state
         self.pressure = pressure
         
         self.inited = False
         self.center = [0, 0, 0]
         self.indices = []

    def onEvent(self, event):
         """This function is the fallback one that is called if the XXXX event is
            received but there is not overriden onXXXX() method.
         """
         #print("generic event handler catched ", event)

    def onAnimateBeginEvent(self, event):
         print("onAnimateBeginEvent")

         # Access the position of the particle 
         particles_mecha = self.state.position.value
         
         if (self.inited == False):
             print("onAnimateBeginEvent INIT")
             print('state vector: ' + str(particles_mecha.size))
             
             for i in range(len(particles_mecha)): 
                self.center += particles_mecha[i]
             
             #print('self.center: ' + str(self.center))
             self.center /= particles_mecha.size
             print('self.center: ' + str(self.center))
             
             
             #self.pressure.pressure = self.center
             a=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
             #np.resize(a,(2,3))
             self.pressure.indices = a
             forces = [self.center, self.center, self.center, self.center, self.center, self.center, self.center, self.center, self.center, self.center]
             
             for i in range(len(self.pressure.indices)): 
                print('self.center: ' + str(self.center - particles_mecha[self.pressure.indices[i]]))
                forces[i] = (self.center - particles_mecha[self.pressure.indices[i]])*10
             #self.pressure.forces = [self.center - particles_mecha[i]]
             #self.pressure.forces.add(self.center - particles_mecha[i])
             #self.pressure.forces.add(self.center - particles_mecha[i])
             self.pressure.forces = forces
             print('test: ' + str(self.pressure.indices[2]))
             print('type: ' +str(type(self.center)))
             self.inited = True
             
             

         print('====================================')
         print('State of the particle')
         print('====================================')
         #print('state vector: '+Length(particles_mecha))

         # Change the the value of the position in z direction
         #with self.state.position.writeable() as state:
         #for i in range(len(self.pressure.)): 
         #state[i][2] += 0.01
             #print('state vector after modification: '+str(particles_mecha[i]))
                     


meshPath = os.path.dirname(os.path.abspath(__file__))+'/mesh/'
plugins=["SofaPython3",
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
    rootNode.addObject('VisualStyle', displayFlags='showCollisionModels hideForceFields showWireframe')

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
    
    #---------------------------------------------------------------------------------------
   


    #----------------------------------------------------------------------------------------
    #Add the liver node
   
    liver = rootNode.addChild('liver')
    
    liver.addObject('MeshVTKLoader', name="loader", filename=meshPath+'liverFine.vtu')
    liver.addObject('TetrahedronSetTopologyContainer', name='liver_topo' , src="@loader")
    MO = liver.addObject('MechanicalObject', name="MO")
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
    collision.addObject('PointCollisionModel', name='pointCol',contactStiffness="1000")
    
    #pressure = collision.addObject('TrianglePressureForceField', name='trianglePressure', template='Vec3d', showForces=True, triangleList='0 1', pressure='0 0 1000')
    ff = collision.addObject('ConstantForceField', forces=[0, 0, 0])
    collision.addObject(MOController(state=MO, pressure = ff))
    

    # Add an actuator for liver
    actuatorLiver = rootNode.addChild('actuatorLiver')
    actuatorLiver.addObject('MechanicalObject', name = 'actuatorLiver', position = '@liver/MO.position', template = 'Vec3d')

    #Add an actuator for particles
    #actuatorParticles = rootNode.addChild('actuatorParticles')
    #actuatorParticles.addObject('MechanicalObject', name = 'actuatorParticles', position = '@carvingElement/Particles.position', template = 'Vec3d')
