import Sofa
import Sofa.Core
import os

meshPath= os.path.dirname(os.path.abspath(__file__))+'/mesh/'


Plugins=["SofaPython3","SoftRobots","ModelOrderReduction","STLIB",
        
         "Sofa.Component.Collision.Detection.Algorithm",
         "Sofa.Component.Collision.Detection.Intersection",
         "Sofa.Component.Collision.Geometry",
         "Sofa.Component.Collision.Response.Contact",
         "Sofa.Component.Visual",
         "Sofa.Component.AnimationLoop",
         "Sofa.GL.Component.Rendering3D",
         "Sofa.Component.Constraint.Lagrangian.Solver",
         'Sofa.Component.IO.Mesh',
         'Sofa.Component.Playback',
         'Sofa.Component.Constraint.Lagrangian.Correction', # Needed to use components [GenericConstraintCorrection]
         'Sofa.Component.Engine.Select', # Needed to use components [BoxROI]
         'Sofa.Component.LinearSolver.Direct', # Needed to use components [SparseLDLSolver]
         'Sofa.Component.LinearSolver.Iterative', # Needed to use components [CGLinearSolver] 
         'Sofa.Component.Mapping.Linear', # Needed to use components [BarycentricMapping]
         'Sofa.Component.Mapping.NonLinear', # Needed to use components [RigidMapping]
         'Sofa.Component.Mass', # Needed to use components [UniformMass]
         'Sofa.Component.ODESolver.Backward', # Needed to use components [EulerImplicitSolver]
         'Sofa.Component.SolidMechanics.FEM.Elastic', # Needed to use components [TetrahedronFEMForceField]
         'Sofa.Component.SolidMechanics.Spring', # Needed to use components [RestShapeSpringsForceField]
         'Sofa.Component.StateContainer', # Needed to use components [MechanicalObject]
         'Sofa.Component.Topology.Container.Dynamic', # Needed to use components [TetrahedronSetTopologyContainer]
         'Sofa.Component.Topology.Container.Constant', # Needed to use components [MeshTopology]
         'Sofa.Component.Constraint.Projective', # Needed to use components [FixedProjectiveConstraint]
         'Sofa.Component.Topology.Mapping',  # Needed to use components [Hexa2TetraTopologicalMapping,Tetra2TriangleTopologicalMapping]
         'Sofa.Component.SceneUtility', # Needed to use components [AddDataRepository]
         'InfinyToolkit',
         'Sofa.Component.Topology.Container.Grid'] # Needed to use components [RegularGridTopology]
        

def createScene(rootNode):
    rootNode.findData('dt').value=0.01
    rootNode.findData('gravity').value=[1, -9, 0]
    
    rootNode.addObject('RequiredPlugin', pluginName=Plugins, printLog=False)

    rootNode.addObject('VisualStyle', displayFlags='hideVisual showBehaviorModels showForceFields')
    
    #rootNode.addObject('AddDataRepository', path='../../../Resources/')
    
    rootNode.addObject('DefaultVisualManagerLoop')

    rootNode.addObject('FreeMotionAnimationLoop')

    #rootNode.addObject('LCPConstraintSolver', tolerance=0.001, maxIt=1000)

    # Collision pipeline
    rootNode.addObject('CollisionPipeline', verbose=0, depth=6)
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('CollisionResponse', name='response' , response='FrictionContactConstraint')
    rootNode.addObject('MinProximityIntersection', name='Proximity', alarmDistance=5.0, contactDistance=1.0)
    
    # Add Organs node
    # Organs = rootNode.addChild('Organs')
    # Organs.addObject('EulerImplicitSolver', name='cg_odesolver', printLog='false',  rayleighStiffness=0.1, rayleighMass=0.0)
    # Organs.addObject('CGLinearSolver', iterations=20, name='linear solver', tolerance=1.0e-8, threshold=1.0e-8 )

    # Add the P20_liver node
    # liver = Organs.addChild('P20_liver')
    #------------------------------------------
    liver = rootNode.addChild('liver')
    liver.addObject('EulerImplicitSolver', name='cg_odesolver', printLog='false',  rayleighStiffness=0.1, rayleighMass=0.0)
    #P20_liver.addObject('CGLinearSolver', iterations=20, name='linear solver', tolerance=1.0e-8, threshold=1.0e-8 )
    #------------------------------------------
    liver.addObject('SparseLDLSolver',template="CompressedRowSparseMatrixMat3x3d")
    #------------------------------------------

    liver.addObject('MeshGmshLoader', name='loader', filename=meshPath+'liver_lowPoly.msh')
    #------------------------------------------
    #liver.addObject('MeshVTKLoader', name="loader", filename=meshPath+'liverFine.vtu')
    #------------------------------------------
    liver.addObject('TetrahedronSetTopologyContainer', name='Container', src='@loader')
    liver.addObject('MechanicalObject', name='dofs') # it should not be link to the loader
    liver.addObject('BoxROI', name='ROI1', box='-100 -56 -240  10 4 -150', drawBoxes=0)
    
    liver.addObject('TetrahedronSetTopologyModifier', name='Modifier')
    liver.addObject('TetrahedronSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d', drawTetrahedra=0, drawScaleTetrahedra=0.7)      
    
    liver.addObject('UniformMass', totalMass=1.0)
   
    liver.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', computeGlobalMatrix='false', method='large', poissonRatio=0.3, youngModulus=3000)

    
    liver.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness = 1e8)

    # Add a collision model

    # Add a visual model
    # liver_visu = liver.addChild('liver_visu')
    # liver_visu.addObject('MeshOBJLoader',name='loaderVisu', filename='mesh/Patient_20/liver_lowPoly.obj', handleSeams=1, scale3d='@../loader.scale3d', translation='@../loader.translation')
    # liver_visu.addObject('NearestTexcoordsMap',name='mapTexcoords', inputPositions='@../loader.position' 
    #                 ,mapPositions='@loaderVisu.position', mapTexCoords='@loaderVisu.texcoords')
    # liver_visu.addObject('OglModel', name='Visual', texturename='Materials/Textures/liver_version4/diffuse_version4.png', texcoords='@mapTexcoords.outputTexCoords')
    # liver_visu.addObject('IdentityMapping', input='@../dofs', output='@Visual')                

    # Add actuator just for the plugin, but it is not needed
    actuatorLiver = rootNode.addChild('actuatorLiver')
    actuatorLiver.addObject('MechanicalObject', name = 'actuatorLiver', position = '@liver/dofs.position', template = 'Vec3d')   
 
            


