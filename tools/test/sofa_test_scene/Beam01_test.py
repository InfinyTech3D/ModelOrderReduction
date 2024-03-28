# Required import for python
import Sofa
import SofaRuntime
import Sofa.Gui


def main():
	# Make sure to load all SOFA libraries
	SofaRuntime.importPlugin("SofaBaseMechanics")

	# Call the above function to create the scene graph
	root = Sofa.Core.Node("root")
	createScene(root)

	# Once defined, initialization of the scene graph
	Sofa.Simulation.init(root)

	# Run the simulation for 10 steps
	for iteration in range(10):
		print(f'Iteration #{iteration}')
		Sofa.Simulation.animate(root, root.dt.value)

	print("Simulation made 10 time steps. Done")
    #print("time is = "+str(root.time.value))


# Function called when the scene graph is being created
def createScene(root):

    root.gravity=[0, 0, -0.9]
    root.name="root"
    root.dt=0.1

   
    root.addObject('RequiredPlugin', name="loadSOFAModules", pluginName="Sofa.Component.LinearSolver.Iterative Sofa.Component.Mass Sofa.Component.MechanicalLoad Sofa.Component.StateContainer Sofa.Component.ODESolver.Backward Sofa.Component.Visual Sofa.Component.AnimationLoop Sofa.Component.Constraint.Lagrangian.Solver")
    
    
    root.addObject('VisualStyle')
    root.VisualStyle.displayFlags="showForceFields hideCollisionModels showBehaviorModels"

    
    # Collision pipeline
    root.addObject('DefaultPipeline')
    root.addObject('FreeMotionAnimationLoop')
    root.addObject('GenericConstraintSolver', resolutionMethod="UnbuildGaussSeidel", tolerance="1e-3", maxIt="200",   printLog="0")
    root.addObject('BruteForceBroadPhase', name="N2")
    root.addObject('BVHNarrowPhase')
    root.addObject('RuleBasedContactManager', responseParams="mu="+str(0.0), name='Response', response='FrictionContactConstraint')# I copied from SofaPython3 doc
    root.addObject('MinProximityIntersection', name="Proximity", alarmDistance="0.75", contactDistance="0.1")# I am not sure about this linw
    root.addObject('LocalMinDistance', alarmDistance=10, contactDistance=5, angleCone=0.01)

    # Topology
    
    

    #  Beam_01
    childNode1 = root.addChild("Beam_01")

    # Time integration scheme and solver
    childNode1.addObject('EulerImplicitSolver', name="cg_odesolver", printLog="false")
    childNode1.addObject('SparseLDLSolver', name="Torus1_SparseLDLSolver", printLog="false") # Assuming that matrix A is linear
    childNode1.addObject('GenericConstraintCorrection', linearSolver='@Torus1_SparseLDLSolver')
    
    childNode1.addObject('RegularGridTopology',name="container", nx="5", ny="20", nz="5", xmin="0", xmax="10", ymin="0", ymax="40", zmin="20", zmax="30")
    childNode1.addObject('MechanicalObject', name="Volume")
   
    childNode1.addObject('DiagonalMass', massDensity="2.0") # the Beam does not move with the uniform mass
    
    childNode1.addObject('BoxROI', name="ROI1", box="-1 -1 0 10 1 50", drawBoxes="1")
    childNode1.addObject('FixedProjectiveConstraint', indices="@ROI1.indices")
    
    # Adding FEM force field
    childNode1.addObject('HexahedronFEMForceField', name="FEM", youngModulus="4000", poissonRatio="0.3", method="large")
    
    
    # Add collision model for Beam_01
    collision = childNode1.addChild('collision')
    collision.addObject('QuadSetTopologyContainer', name="collisionModel")
    collision.addObject('QuadSetTopologyModifier',  name="Modifier")
    collision.addObject('QuadSetGeometryAlgorithms', name="GeomAlgo", template="Vec3d")
    collision.addObject('Hexa2QuadTopologicalMapping', input="@../container", output="@collisionModel")
    collision.addObject('TriangleCollisionModel')
    collision.addObject('PointCollisionModel')

    # Add a visual model
    visualModel = childNode1.addChild('visualModel')
    visualModel.addObject('OglModel', name="visu", color="green")
    visualModel.addObject('IdentityMapping', input="@..", output="@visu")

    # Add an actuator for Beam_01
    actuator_Beam01 = root.addChild('actuator_Beam01')
    actuator_Beam01.addObject('MechanicalObject', name="actuatedBeam01", template="Vec3d", position="@./Beam_01/Volume.position")

    ################################################################
   
    

    return root


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
