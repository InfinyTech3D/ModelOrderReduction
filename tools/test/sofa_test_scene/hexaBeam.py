# -*- coding: utf-8 -*-
import os
import Sofa

plugins=["SofaPython3","SoftRobots","ModelOrderReduction","STLIB",
        
         "Sofa.Component.Visual",
         "Sofa.Component.AnimationLoop",
         "Sofa.GL.Component.Rendering3D",
         "Sofa.Component.Constraint.Lagrangian.Solver",
         'Sofa.Component.IO.Mesh',
         'Sofa.Component.Playback',
         'Sofa.Component.Constraint.Lagrangian.Correction', # Needed to use components [GenericConstraintCorrection]
         'Sofa.Component.Engine.Select', # Needed to use components [BoxROI]
         'Sofa.Component.LinearSolver.Direct', # Needed to use components [SparseLDLSolver]
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

    rootNode.findData('dt').value=0.02
    rootNode.findData('gravity').value=[0, -9810, 0]
    rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels showForceFields')
    rootNode.addObject('RequiredPlugin', pluginName=plugins, printLog=False)
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance="1e-12", maxIterations="10000")


    M1 = rootNode.addChild("M1")
    M1.addObject('EulerImplicitSolver',rayleighStiffness="0.1", rayleighMass="0.1")
    M1.addObject('SparseLDLSolver',name='preconditioner')
    M1.addObject('MechanicalObject')
    M1.addObject('UniformMass', totalMass="0.1")
    M1.addObject('RegularGridTopology', nx="4", ny="4", nz="20", xmin="-9", xmax="-6", ymin="0", ymax="3", zmin="0", zmax="19")
    M1.addObject('BoxROI', name='ROI1', box='-9 -2 -1 -3 4 5', drawBoxes='true')

    M1.addObject('HexahedronFEMForceField', name="FEM", youngModulus="4000", poissonRatio="0.3", method="large")
    M1.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness = '1e8')
    M1.addObject('GenericConstraintCorrection', linearSolver='@preconditioner')
    
    cableNodeTip = M1.addChild('cableNodeTip')
    cableNodeTip.addObject('MechanicalObject', name="actuatedTip", template="Vec3d", position="-7.5 1.5 18.5")
    cableNodeTip.addObject('CableConstraint', name="tipCable", indices="0", pullPoint="-7.5 10 18.5", valueType="displacement")
    cableNodeTip.addObject('BarycentricMapping', mapForces="false", mapMasses="false")
    
    cableNodeSide = M1.addChild('cableNodeSide')
    cableNodeSide.addObject('MechanicalObject', name="actuatedSide", template="Vec3d", position="-7.5 1.5 18.5")
    cableNodeSide.addObject('CableConstraint', name="sideCable", indices="0", pullPoint="1 10 18.5", valueType="displacement")
    cableNodeSide.addObject('BarycentricMapping', mapForces="false", mapMasses="false")
    

