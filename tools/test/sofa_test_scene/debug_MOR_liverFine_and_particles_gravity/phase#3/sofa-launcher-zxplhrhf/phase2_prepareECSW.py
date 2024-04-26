# -*- coding: utf-8 -*-
import sys
import numpy as np

#   STLIB IMPORT
try:
    from splib3.animation import AnimationManager , animate
    from stlib3.scene.wrapper import Wrapper
    from splib3.scenegraph import *

except:
    raise ImportError("ModelOrderReduction plugin depend on SPLIB"\
                     +"Please install it : https://github.com/SofaDefrost/STLIB")

# MOR IMPORT
from mor.utility import sceneCreation as u
from mor.wrapper import replaceAndSave

# Our Phase1 Scene IMPORT
import phase1_snapshots

# Scene parameters
phase = []
phase.append(0)
nbrOfModes = 9
periodSaveGIE = 6
paramWrapper = ('/liver', {'paramForcefield': {'prepareECSW': True, 'modesPath': 'C:\\projects\\sofa_dev\\sofa_plugins\\MOR\\tools\\test\\sofa_test_scene\\liverFine_and_particles_with_gravity_Res\\data\\modes.txt', 'periodSaveGIE': 6, 'nbTrainingSet': 15}, 'paramMORMapping': {'input': '@../MechanicalObject', 'modesPath': 'C:\\projects\\sofa_dev\\sofa_plugins\\MOR\\tools\\test\\sofa_test_scene\\liverFine_and_particles_with_gravity_Res\\data\\modes.txt'}, 'paramMappedMatrixMapping': {'nodeToParse': '@.//liver', 'template': 'Vec1d,Vec1d', 'object1': '@./MechanicalObject', 'object2': '@./MechanicalObject', 'timeInvariantMapping1': True, 'timeInvariantMapping2': True, 'performECSW': False}})
phaseToSave = [0]

path, param = paramWrapper
param['nbrOfModes'] = 9


def createScene(rootNode):

    # Import Original Scene with the animation added
    # Here we use a wrapper (MORWrapper) that will allow us (with MORreplace)
    # to modify the initial scene and get informations on its structures
    # For more details on the process involved additionnal doc are with :
    #       - mor.wrapper.MORWrapper
    #       - mor.script.sceneCreationUtility

    phase1_snapshots.createScene(Wrapper(rootNode, replaceAndSave.MORreplace, paramWrapper))

    # Add MOR plugin if not found
    #u.addPlugin(rootNode,"ModelOrderReduction")

    # Save connectivity list that will allow us after work only on the necessary elements

    if phase == phaseToSave:
        u.saveElements(rootNode,rootNode.dt,replaceAndSave.forcefield)
        param['paramMappedMatrixMapping']['saveReducedMass'] = True

    # Modify the scene to perform hyper-reduction according
    # to the informations collected by the wrapper

    u.modifyGraphScene(rootNode,nbrOfModes,paramWrapper)


    # We Update the link 
    for path , item in replaceAndSave.pathToUpdate.items():
        data , newValue = item
        obj = get(rootNode,path)
        setattr(obj,data,newValue)