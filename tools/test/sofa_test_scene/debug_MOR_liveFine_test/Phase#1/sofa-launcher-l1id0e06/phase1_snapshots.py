# -*- coding: utf-8 -*-
import os
import platform
import importlib
from importlib.util import spec_from_loader, module_from_spec
from importlib.machinery import SourceFileLoader 

#   STLIB IMPORT
try:
    from splib3.animation import AnimationManager , animate
    from stlib3.scene.wrapper import Wrapper
    from splib3.scenegraph import *
except:
    raise ImportError("ModelOrderReduction plugin depend on SPLIB"\
                     +"Please install it : https://github.com/SofaDefrost/STLIB")

# MOR IMPORT
from mor.reduction.container import ObjToAnimate
from mor.utility import sceneCreation as u

slash = '/'
if "Windows" in platform.platform():
    slash = "\\"

# Import animation
fileToImport = os.path.normpath(r'C:\projects\sofa_dev\sofa-build\lib\python3\site-packages\mor\reduction\container/../../animation/shakingAnimations.py')
scene = fileToImport.split(slash)[-1].split('.')[0]
os.sys.path.insert(0,os.path.dirname(os.path.abspath(fileToImport)))
animationFct = importlib.import_module(scene)

# is there an __all__?  if so respect it
if "__all__" in animationFct.__dict__:
    names = animationFct.__dict__["__all__"]
else:
    # otherwise we import all names that don't begin with _
    names = [x for x in animationFct.__dict__ if not x.startswith("_")]

# now drag them in
globals().update({k: getattr(animationFct, k) for k in names})

# Our Original Scene IMPORT
pathScene = r'C:\projects\sofa_dev\sofa_plugins\MOR\tools\test\sofa_test_scene\liverFine_test.py'
scene = pathScene.split(slash)[-1]
spec = spec_from_loader(scene, SourceFileLoader(scene, pathScene))
originalScene = module_from_spec(spec)
spec.loader.exec_module(originalScene)


# Animation parameters
listObjToAnimate = []
listObjToAnimate.append(ObjToAnimate('actuator/actuatorState',doingNothing,duration=-1,**{'incr': 0.4, 'incrPeriod': 2.5, 'rangeOfAction': 6.2, 'angle': 0, 'rodRadius': 0.4}))
phase = []
phase.append(0)
nbIterations = 41
paramWrapper = ('/liver', {'paramForcefield': {'prepareECSW': True, 'modesPath': 'C:\\projects\\sofa_dev\\sofa_plugins\\MOR\\tools\\test\\sofa_test_scene\\liverFine-test_Res\\data\\modes.txt', 'periodSaveGIE': 6, 'nbTrainingSet': 15}, 'paramMORMapping': {'input': '@../MechanicalObject', 'modesPath': 'C:\\projects\\sofa_dev\\sofa_plugins\\MOR\\tools\\test\\sofa_test_scene\\liverFine-test_Res\\data\\modes.txt'}, 'paramMappedMatrixMapping': {'nodeToParse': '@.//liver', 'template': 'Vec1d,Vec1d', 'object1': '@./MechanicalObject', 'object2': '@./MechanicalObject', 'timeInvariantMapping1': True, 'timeInvariantMapping2': True, 'performECSW': False}})
phaseToSave = [0]


def createScene(rootNode):
    print ("Scene Phase :"+str(phase))

    # Import Original scene

    originalScene.createScene(rootNode)
    dt = rootNode.dt.value
    timeExe = nbIterations * dt

    # Add Animation Manager to Scene
    # (ie: python script controller to which we will pass our differents animations)
    # more details at splib.animation.AnimationManager (https://stlib.readthedocs.io/en/latest/)

    if isinstance(rootNode, Wrapper):
        rootNode.addObject(AnimationManager(rootNode.node))
    else:
        rootNode.addObject(AnimationManager(rootNode))

    # Now that we have the AnimationManager & a list of the nodes we want to animate
    # we can add an animation to then according to the arguments in listObjToAnimate

    u.addAnimation(rootNode,phase,timeExe,dt,listObjToAnimate)

    # Now that all the animations are defined we need to record their results
    # for that we take the parent node normally given as an argument in paramWrapper

    path, param = paramWrapper
    myParent = get(rootNode,path[1:])

    # We need rest_position and because it is normally always the same we record it one time
    # during the first phase with the argument writeX0 put to True
    if phase == phaseToSave:
        myParent.addObject('WriteState', filename="stateFile.state",period=listObjToAnimate[0].params["incrPeriod"]*dt,
                                            writeX="1", writeX0="1", writeV="0")
    else :
        myParent.addObject('WriteState', filename="stateFile.state", period=listObjToAnimate[0].params["incrPeriod"]*dt,
                                            writeX="1", writeX0="0", writeV="0")

    # If you want to save also the velocity uncomment the what is below.
    # Then after if you give to **ReduceModel** *saveVelocitySnapshots = True* as parameter
    # all the different velocity saved will be added to one file as the stateFile

    # myParent.addObject('WriteState', filename="stateFileVelocity.state",period=listObjToAnimate[0].params["incrPeriod"]*dt,
    #                                       writeX = "0", writeX0 = "0", writeV = "1")
