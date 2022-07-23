#!/usr/bin/env python
from gym.envs.registration import register
from gym import envs


def RegisterOpenAI_Ros_Env(task_env, max_episode_steps=10000):
    """
    Registers all the ENVS supported in OpenAI ROS. This way we can load them
    with variable limits.
    Here is where you have to PLACE YOUR NEW TASK ENV, to be registered and accesible.
    return: False if the Task_Env wasnt registered, True if it was.
    """

    ###########################################################################
    # MovingCube Task-Robot Envs

    result = True

    # Cubli Moving Cube
    if task_env == 'MovingCubeOneDiskWalk-v0':
        print("Import module")

        # We have to import the Class that we registered so that it can be found afterwards in the Make
        from openai_ros.task_envs.moving_cube import one_disk_walk

        print("Importing register env")
        # We register the Class through the Gym system
        register(
            id=task_env,
            #entry_point='openai_ros:task_envs.moving_cube.one_disk_walk.MovingCubeOneDiskWalkEnv',
            entry_point='openai_ros.task_envs.moving_cube.one_disk_walk:MovingCubeOneDiskWalkEnv',
            max_episode_steps=max_episode_steps,
        )


    # Husarion Robot
    elif task_env == 'HusarionGetToPosTurtleBotPlayGround-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.husarion.husarion_get_to_position_turtlebot_playground:HusarionGetToPosTurtleBotPlayGroundEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.husarion import husarion_get_to_position_turtlebot_playground

    elif task_env == 'FetchTest-v0':
        register(
            id=task_env,
            entry_point='openai_ros.task_envs.fetch.fetch_test_task:FetchTestEnv',
            max_episode_steps=max_episode_steps,
        )
        # 50
        # We have to import the Class that we registered so that it can be found afterwards in the Make
        from openai_ros.task_envs.fetch import fetch_test_task

    elif task_env == 'FetchSimpleTest-v0':
        register(
            id=task_env,
            # entry_point='openai_ros:task_envs.fetch.fetch_simple_task.FetchSimpleTestEnv',
            entry_point='openai_ros.task_envs.fetch.fetch_simple_task:FetchSimpleTestEnv',
            max_episode_steps=max_episode_steps,
        )
        # We have to import the Class that we registered so that it can be found afterwards in the Make
        from openai_ros.task_envs.fetch import fetch_simple_task

    elif task_env == 'FetchPickAndPlace-v0':
        register(
            id=task_env,
            # entry_point='openai_ros:task_envs.fetch.fetch_pick_and_place_task.FetchPickAndPlaceEnv',
            entry_point='openai_ros.task_envs.fetch.fetch_pick_and_place_task:FetchPickAndPlaceEnv',
            max_episode_steps=max_episode_steps,
        )
        # We have to import the Class that we registered so that it can be found afterwards in the Make
        from openai_ros.task_envs.fetch import fetch_pick_and_place_task

    elif task_env == 'FetchPush-v0':
        register(
            id=task_env,
            # entry_point='openai_ros:task_envs.fetch.fetch_pick_and_place_task.FetchPushEnv',
            # entry_point='openai_ros:task_envs.fetch.fetch_push.FetchPushEnv',
            entry_point='openai_ros.task_envs.fetch.fetch_push:FetchPushEnv',
            max_episode_steps=max_episode_steps,
        )
        # We have to import the Class that we registered so that it can be found afterwards in the Make
        from openai_ros.task_envs.fetch import fetch_push

    elif task_env == 'CartPoleStayUp-v0':
        register(
            id=task_env,
            entry_point='openai_ros.task_envs.cartpole_stay_up.stay_up:CartPoleStayUpEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.cartpole_stay_up import stay_up

    elif task_env == 'HopperStayUp-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.hopper.hopper_stay_up:HopperStayUpEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.hopper import hopper_stay_up

    elif task_env == 'IriWamTcpToBowl-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.iriwam.tcp_to_bowl:IriWamTcpToBowlEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.iriwam import tcp_to_bowl

    elif task_env == 'ParrotDroneGoto-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.parrotdrone.parrotdrone_goto:ParrotDroneGotoEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.parrotdrone import parrotdrone_goto

    elif task_env == 'SawyerTouchCube-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.sawyer.learn_to_touch_cube:SawyerTouchCubeEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.sawyer import learn_to_touch_cube

    elif task_env == 'ShadowTcGetBall-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.shadow_tc.learn_to_pick_ball:ShadowTcGetBallEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.shadow_tc import learn_to_pick_ball

    elif task_env == 'SumitXlRoom-v0':

        register(
            id='SumitXlRoom-v0',
            entry_point='openai_ros.task_envs.sumit_xl.sumit_xl_room:SumitXlRoom',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.sumit_xl import sumit_xl_room

    elif task_env == 'MyTurtleBot2Maze-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.turtlebot2.turtlebot2_maze:TurtleBot2MazeEnv',
            max_episode_steps=max_episode_steps,
        )
        # import our training environment
        from openai_ros.task_envs.turtlebot2 import turtlebot2_maze

    elif task_env == 'MyTurtleBot2Wall-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.turtlebot2.turtlebot2_wall:TurtleBot2WallEnv',
            max_episode_steps=max_episode_steps,
        )
        # import our training environment
        from openai_ros.task_envs.turtlebot2 import turtlebot2_wall

    elif task_env == 'UnitreeDogWorld-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.unitreeDog.unitree_task:UnitreeDogTaskEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.unitreeDog import unitree_task

    elif task_env == 'TurtleBot3World-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.turtlebot3.turtlebot3_world:TurtleBot3WorldEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.turtlebot3 import turtlebot3_world
        
    elif task_env == 'WamvNavTwoSetsBuoys-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.wamv.wamv_nav_twosets_buoys:WamvNavTwoSetsBuoysEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.wamv import wamv_nav_twosets_buoys

    # Add here your Task Envs to be registered
    else:
        result = False

    ###########################################################################

    if result:
        # We check that it was really registered
        supported_gym_envs = GetAllRegisteredGymEnvs()
        #print("REGISTERED GYM ENVS===>"+str(supported_gym_envs))
        assert (task_env in supported_gym_envs), "The Task_Robot_ENV given is not Registered ==>" + \
            str(task_env)

    return result


def GetAllRegisteredGymEnvs():
    """
    Returns a List of all the registered Envs in the system
    return EX: ['Copy-v0', 'RepeatCopy-v0', 'ReversedAddition-v0', ... ]
    """

    all_envs = envs.registry.all()
    env_ids = [env_spec.id for env_spec in all_envs]

    return env_ids
