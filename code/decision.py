import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    if Rover.started == False:
        Rover.started = True
        Rover.start_position = (Rover.pos[0], Rover.pos[1])
    if Rover.samples_collected >5:
        Rover.finished = True
    #TODO: when finished it can go back to start position by using dynamic programming

    if Rover.picking_up == 1:
        Rover.samples_collected += 1
        Rover.sample_on_sight = False
        Rover.picking_up = 0

        Rover.brake = 0
        Rover.steer = 45
        Rover.throttle = Rover.throttle_set
        Rover.mode = 'reverse'
        return Rover

    if Rover.near_sample == True:
        Rover.mode = 'stop'
        Rover.brake = Rover.brake_set
        Rover.throttle = 0
        Rover.send_pickup = True

        Rover.sample_on_sight = False
        Rover.sample_persistance = 0
        Rover.sample_angles = None
        Rover.sample_dists = None

        return Rover

    if Rover.sample_on_sight is True and Rover.sample_persistance > 3:
        Rover.mode = 'approach'

    if Rover.vel < 0.2:
        Rover.stuck_frames += 1
    if Rover.stuck_frames > 50:
        Rover.Mode = 'stop'

    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0

                #TODO: depends on the two wide always left. how to wall follow?
                # if np.max(Rover.nav_angles) - np.min(nav_angles)> 90:
                #     pass
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -8, 8)
            # If there's a lack of navigable terrain pixels then go to 'stop' ode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'


        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:

                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    #TODO: depends on the Obstacle perception steer where obstacles

                     # Could be more clever here about which way to turn
                    if len(Rover.obstacle_dists) < Rover.go_forward/10:
                        Rover.mode = 'reverse'
                        Rover.steer = 0
                    else:
                        Rover.steer = 22.5

                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.stuck_frames = 0
                    Rover.mode = 'forward'

        elif Rover.mode == 'reverse':
            if len(Rover.obstacle_dists)>0:
                Rover.brake = 0
                Rover.throttle = -1
                Rover.steer = 0
                if np.min(Rover.obstacle_dists) > 40:
                    Rover.mode = 'foward'
                else:
                    Rover.mode = 'reverse'
            else:
                Rover.mode = 'stop'

        elif Rover.mode == 'approach':
            if len(Rover.sample_angles)> 0 :

                rock_distance = np.mean(Rover.sample_dists)
                rock_angle = np.mean(Rover.sample_angles * 180 / np.pi)
                if len(Rover.nav_angles) >= Rover.stop_forward:
                    if rock_distance > 30:
                        if Rover.vel > 3:
                            Rover.brake = rock_distance/100
                        else:
                            Rover.brake = 0
                        Rover.throttle = Rover.throttle_set
                        Rover.steer = rock_angle/2
                        # if Rover.send_pickup is True:
                        #     Rover.send_pickup = False
                    elif rock_distance < 30:
                        if Rover.vel > 1:
                            Rover.brake = rock_distance/40
                        else:
                            Rover.brake = 0
                        Rover.throttle = Rover.throttle_set/2
                        Rover.steer = rock_angle*2

                        print("Sample in proximity thr, st, Br, near_sample: ",Rover.throttle,Rover.steer,Rover.brake, Rover.near_sample)
                elif len(Rover.nav_angles) < Rover.stop_forward:
                    Rover.mode = 'stop'
                    return Rover


            else:
                if Rover.sample_persistance < 1:
                     Rover.sample_on_sight = False
                     Rover.mode = 'forward'
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    print(Rover.mode,Rover.pos)
    return Rover
