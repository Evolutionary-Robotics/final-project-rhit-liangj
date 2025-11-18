import pyrosim.pyrosim as pyrosim


def World(n,xnum,ynum):
    pyrosim.Start_SDF("boxes.sdf")
    for x in range(xnum):
        for y in range(ynum):
            for i in range(n):
                s = 1 - i/n
                pyrosim.Send_Cube(name="Box", pos=[x,y,0.5 + i] , size=[s,s,s])
    pyrosim.End()

def Robot_4_leg():
    pyrosim.Start_URDF("robot.urdf")

    #body
    pyrosim.Send_Cube(name="body", pos=[0, 0, 1.25], size=[1, 0.5, 0.3])

    # parameters
    upper_leg_length = 0.5
    lower_leg_length = 0.5
    leg_width = 0.1

    # LEG 1
    pyrosim.Send_Joint(name="body_leg1_upper", parent="body", child="leg1_upper",
                       type="revolute", position=[0.5, 0.25, 1.1])
    pyrosim.Send_Cube(name="leg1_upper", pos=[0, 0, -upper_leg_length/2], size=[leg_width, leg_width, upper_leg_length])

    pyrosim.Send_Joint(name="leg1_upper_leg1_lower", parent="leg1_upper", child="leg1_lower",
                       type="revolute", position=[0, 0, -upper_leg_length])
    pyrosim.Send_Cube(name="leg1_lower", pos=[0, 0, -lower_leg_length/2], size=[leg_width, leg_width, lower_leg_length])

    # LEG 2
    pyrosim.Send_Joint(name="body_leg2_upper", parent="body", child="leg2_upper",
                       type="revolute", position=[0.5, -0.25, 1.1])
    pyrosim.Send_Cube(name="leg2_upper", pos=[0, 0, -upper_leg_length/2], size=[leg_width, leg_width, upper_leg_length])

    pyrosim.Send_Joint(name="leg2_upper_leg2_lower", parent="leg2_upper", child="leg2_lower",
                       type="revolute", position=[0, 0, -upper_leg_length])
    pyrosim.Send_Cube(name="leg2_lower", pos=[0, 0, -lower_leg_length/2], size=[leg_width, leg_width, lower_leg_length])

    # LEG 3
    pyrosim.Send_Joint(name="body_leg3_upper", parent="body", child="leg3_upper",
                       type="revolute", position=[-0.5, 0.25, 1.1])
    pyrosim.Send_Cube(name="leg3_upper", pos=[0, 0, -upper_leg_length/2], size=[leg_width, leg_width, upper_leg_length])

    pyrosim.Send_Joint(name="leg3_upper_leg3_lower", parent="leg3_upper", child="leg3_lower",
                       type="revolute", position=[0, 0, -upper_leg_length])
    pyrosim.Send_Cube(name="leg3_lower", pos=[0, 0, -lower_leg_length/2], size=[leg_width, leg_width, lower_leg_length])

    # LEG 4
    pyrosim.Send_Joint(name="body_leg4_upper", parent="body", child="leg4_upper",
                       type="revolute", position=[-0.5, -0.25, 1.1])
    pyrosim.Send_Cube(name="leg4_upper", pos=[0, 0, -upper_leg_length/2], size=[leg_width, leg_width, upper_leg_length])

    pyrosim.Send_Joint(name="leg4_upper_leg4_lower", parent="leg4_upper", child="leg4_lower",
                       type="revolute", position=[0, 0, -upper_leg_length])
    pyrosim.Send_Cube(name="leg4_lower", pos=[0, 0, -lower_leg_length/2], size=[leg_width, leg_width, lower_leg_length])

    pyrosim.End()

def Robot_6_leg():
    pyrosim.Start_URDF("robot2.urdf")

    # Body
    pyrosim.Send_Cube(name="body", pos=[0, 0, 1.25], size=[1.2, 0.5, 0.3])

    # Parameters
    upper_leg_length = 0.5
    lower_leg_length = 0.5
    leg_width = 0.1

    # LEG 1 (front right)
    pyrosim.Send_Joint(name="body_leg1_upper", parent="body", child="leg1_upper",
                       type="revolute", position=[0.6, 0.25, 1.1])
    pyrosim.Send_Cube(name="leg1_upper", pos=[0, 0, -upper_leg_length/2], size=[leg_width, leg_width, upper_leg_length])

    pyrosim.Send_Joint(name="leg1_upper_leg1_lower", parent="leg1_upper", child="leg1_lower",
                       type="revolute", position=[0, 0, -upper_leg_length])
    pyrosim.Send_Cube(name="leg1_lower", pos=[0, 0, -lower_leg_length/2], size=[leg_width, leg_width, lower_leg_length])

    # LEG 2 (middle right)
    pyrosim.Send_Joint(name="body_leg2_upper", parent="body", child="leg2_upper",
                       type="revolute", position=[0.0, 0.25, 1.1])
    pyrosim.Send_Cube(name="leg2_upper", pos=[0, 0, -upper_leg_length/2], size=[leg_width, leg_width, upper_leg_length])

    pyrosim.Send_Joint(name="leg2_upper_leg2_lower", parent="leg2_upper", child="leg2_lower",
                       type="revolute", position=[0, 0, -upper_leg_length])
    pyrosim.Send_Cube(name="leg2_lower", pos=[0, 0, -lower_leg_length/2], size=[leg_width, leg_width, lower_leg_length])

    # LEG 3 (rear right)
    pyrosim.Send_Joint(name="body_leg3_upper", parent="body", child="leg3_upper",
                       type="revolute", position=[-0.6, 0.25, 1.1])
    pyrosim.Send_Cube(name="leg3_upper", pos=[0, 0, -upper_leg_length/2], size=[leg_width, leg_width, upper_leg_length])

    pyrosim.Send_Joint(name="leg3_upper_leg3_lower", parent="leg3_upper", child="leg3_lower",
                       type="revolute", position=[0, 0, -upper_leg_length])
    pyrosim.Send_Cube(name="leg3_lower", pos=[0, 0, -lower_leg_length/2], size=[leg_width, leg_width, lower_leg_length])

    # LEG 4 (front left)
    pyrosim.Send_Joint(name="body_leg4_upper", parent="body", child="leg4_upper",
                       type="revolute", position=[0.6, -0.25, 1.1])
    pyrosim.Send_Cube(name="leg4_upper", pos=[0, 0, -upper_leg_length/2], size=[leg_width, leg_width, upper_leg_length])

    pyrosim.Send_Joint(name="leg4_upper_leg4_lower", parent="leg4_upper", child="leg4_lower",
                       type="revolute", position=[0, 0, -upper_leg_length])
    pyrosim.Send_Cube(name="leg4_lower", pos=[0, 0, -lower_leg_length/2], size=[leg_width, leg_width, lower_leg_length])

    # LEG 5 (middle left)
    pyrosim.Send_Joint(name="body_leg5_upper", parent="body", child="leg5_upper",
                       type="revolute", position=[0.0, -0.25, 1.1])
    pyrosim.Send_Cube(name="leg5_upper", pos=[0, 0, -upper_leg_length/2], size=[leg_width, leg_width, upper_leg_length])

    pyrosim.Send_Joint(name="leg5_upper_leg5_lower", parent="leg5_upper", child="leg5_lower",
                       type="revolute", position=[0, 0, -upper_leg_length])
    pyrosim.Send_Cube(name="leg5_lower", pos=[0, 0, -lower_leg_length/2], size=[leg_width, leg_width, lower_leg_length])

    # LEG 6 (rear left)
    pyrosim.Send_Joint(name="body_leg6_upper", parent="body", child="leg6_upper",
                       type="revolute", position=[-0.6, -0.25, 1.1])
    pyrosim.Send_Cube(name="leg6_upper", pos=[0, 0, -upper_leg_length/2], size=[leg_width, leg_width, upper_leg_length])

    pyrosim.Send_Joint(name="leg6_upper_leg6_lower", parent="leg6_upper", child="leg6_lower",
                       type="revolute", position=[0, 0, -upper_leg_length])
    pyrosim.Send_Cube(name="leg6_lower", pos=[0, 0, -lower_leg_length/2], size=[leg_width, leg_width, lower_leg_length])

    pyrosim.End()

def Robot_3_leg():
    pyrosim.Start_URDF("robot3.urdf")

    # --- MAIN BODY ---
    pyrosim.Send_Cube(name="body", pos=[0, 0, 1.25], size=[0.8, 0.8, 0.3])

    # common parameters
    upper_leg_length = 0.5
    lower_leg_length = 0.5
    leg_width = 0.1

    # Leg joint positions (120 degrees apart)
    # You can adjust these if you want a different shape
    leg_positions = [
        [ 0.5,  0.0, 1.1],   # Leg 1: front
        [-0.25,  0.45, 1.1], # Leg 2: left-back
        [-0.25, -0.45, 1.1]  # Leg 3: right-back
    ]

    # Loop to make all legs
    for i, pos in enumerate(leg_positions, start=1):
        # Hip joint (upper leg)
        pyrosim.Send_Joint(
            name=f"body_leg{i}_upper",
            parent="body",
            child=f"leg{i}_upper",
            type="revolute",
            position=pos
        )
        pyrosim.Send_Cube(
            name=f"leg{i}_upper",
            pos=[0, 0, -upper_leg_length/2],
            size=[leg_width, leg_width, upper_leg_length]
        )

        # Knee joint (lower leg)
        pyrosim.Send_Joint(
            name=f"leg{i}_upper_leg{i}_lower",
            parent=f"leg{i}_upper",
            child=f"leg{i}_lower",
            type="revolute",
            position=[0, 0, -upper_leg_length]
        )
        pyrosim.Send_Cube(
            name=f"leg{i}_lower",
            pos=[0, 0, -lower_leg_length/2],
            size=[leg_width, leg_width, lower_leg_length]
        )

    pyrosim.End()



Robot_4_leg()
Robot_6_leg()
Robot_3_leg()
#World(10,5,5)
