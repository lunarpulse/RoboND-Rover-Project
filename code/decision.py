import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    if Rover.started == False:
        Rover.started = True
        Rover.drive_tendancy = -1
        Rover.start_position = (Rover.pos[0], Rover.pos[1])
    if Rover.samples_collected >5:
        Rover.finished = True
    #TODO: when finished it can go back to start position by using dynamic programming

    if Rover.picking_up == 1:
        Rover.samples_collected += 1
        Rover.sample_on_sight = False
        Rover.picking_up = 0

        Rover.brake = 0
        Rover.steer = 15
        Rover.throttle = Rover.throttle_set
        Rover.mode = 'stop'
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

    if  -0.2 <Rover.vel < 0.1 :
        if Rover.mode is not 'reverse':
            Rover.stuck_frames += 1
    else:
        Rover.stuck_frames = 0

    if Rover.stuck_frames > 50:
        Rover.Mode = 'reverse'


    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # if Rover.stuck_frames > 20:
            #     Rover.steer = np.clip(np.max(Rover.nav_angles * 180/np.pi), -15-Rover.drive_tendancy*Rover.drive_bias_angle, 15-Rover.drive_tendancy*Rover.drive_bias_angle)
            #     Rover.brake = 0
            #     Rover.throttle = 1
            #     return Rover
            if Rover.stuck_frames > 40:
                Rover.mode = 'stop'
                Rover.steer = -15*Rover.drive_tendancy
                Rover.brake = 0
                Rover.throttle = 0

            # Check the extent of navigable terrain
            #TODO wide? 2 way? get the right way
            #if nav_angles wideer than a Threshold than take right or left
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0

                # if Rover.vel> 2.5 and (np.max(Rover.nav_angles* 180 / np.pi) > 25 and np.min(Rover.nav_angles* 180 / np.pi) < -25):
                #     Rover.brake =0.5

                # if Rover.vel < 0.2:
                #     Rover.brake =0
                #     Rover.mode = 'stop'

                #TODO: depends on the two wide always left. how to wall follow?
                # if np.max(Rover.nav_angles) - np.min(nav_angles)> 90:
                #     pass
                # Set steering to average angle clipped to the range +/- 15

                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) + Rover.drive_tendancy* Rover.drive_bias_angle
                if Rover.vel> 1.5 and (np.mean(Rover.obstacle_angles* 180 / np.pi) > 6 or np.mean(Rover.obstacle_angles* 180 / np.pi)< -6):
                    Rover.brake = 0.2
                    Rover.steer = 15 * Rover.drive_tendancy
                    Rover.throttle /= 2
                else:
                    Rover.brake =0

                if len(Rover.obstacle_dists)> 2000:
                    Rover.steer = 15 * Rover.drive_tendancy
                if Rover.vel > 1.5:
                    Rover.steer = Rover.steer/2
                # obs_angle_min = np.min(Rover.obstacle_angles * 180/np.pi)
                # obs_angle_max = np.max(Rover.obstacle_angles * 180/np.pi)
                # if obs_angle_max > 10 and obs_angle_min< -10
                #     if Rover.drive_tendancy >  0:
                #         Rover.steer = np.clip(obs_angle_max, -15, 15) - Rover.drive_tendancy* Rover.drive_bias_angle
                #     else:
                #         Rover.steer = np.clip(obs_angle_min, -15, 15) - Rover.drive_tendancy* Rover.drive_bias_angle
                if Rover.stuck_frames > 15:
                    Rover.throttle = 1
            # If there's a lack of navigable terrain pixels then go to 'stop' ode
            elif len(Rover.nav_angles) < Rover.stop_forward or Rover.stuck_frames > 10:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = 0
                    Rover.steer = Rover.steer = -15*Rover.drive_tendancy
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
                if np.mean(Rover.obstacle_dists) < 50 or len(Rover.nav_angles) < Rover.go_forward:#(np.mean(Rover.obstacle_angles* 180 / np.pi) > 10 or np.mean(Rover.obstacle_angles* 180 / np.pi)< -10):#len(Rover.nav_angles) < Rover.go_forward:

                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    #TODO: depends on the Obstacle perception steer where obstacles

                     # Could be more clever here about which way to turn
                    Rover.steer = -15*Rover.drive_tendancy#np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

                # If we're stopped but see sufficient navigable terrain in front then go!
                if np.mean(Rover.obstacle_dists) >= 50 and len(Rover.nav_angles) >= Rover.go_forward and Rover.stuck_frames < 5: #(-10 < np.mean(Rover.obstacle_angles* 180 / np.pi) < 10):
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                if Rover.stuck_frames > 60:
                    Rover.mode = 'reverse'
                    Rover.steer = 0
                    Rover.brake = 5
                    Rover.throttle = 0
                # if len(Rover.obstacle_dists) < Rover.go_forward/2:
                # if Rover.stuck_frames > 50:
                #     Rover.mode = 'reverse'
                #     Rover.steer = 0
                #     Rover.brake = 5
                #     Rover.throttle = 0
                # else:
                #     pass
        elif Rover.mode == 'reverse':
            if Rover.stuck_frames> 5:
                Rover.brake = 0
                Rover.throttle = -1
                Rover.steer = Rover.drive_tendancy* np.clip(np.mean(Rover.obstacle_angles* 180 / np.pi)*4,-15,15)
                Rover.stuck_frames -= 1
                if np.max(Rover.obstacle_angles) > 40:
                    Rover.mode = 'foward'
                    Rover.brake = 2
                    Rover.throttle = Rover.throttle_set
                else:
                    Rover.mode = 'reverse'
            else:
                Rover.mode = 'forward'

        elif Rover.mode == 'approach':

            if len(Rover.sample_angles)> 0 :

                rock_distance = np.mean(Rover.sample_dists)
                rock_angle = np.mean(Rover.sample_angles * 180 / np.pi)
                if len(Rover.nav_angles) >= Rover.stop_forward:
                    if rock_distance > 30:
                        if Rover.vel > 2:
                            Rover.brake = rock_distance/100
                        else:
                            Rover.brake = 0
                        Rover.throttle = Rover.throttle_set*2
                        Rover.steer = rock_angle
                        if Rover.stuck_frames > 20:
                            Rover.throttle = 1
                            Rover.steer = np.clip(np.mean(Rover.obstacle_angles* 180 / np.pi)*4,-15,15)*Rover.drive_tendancy
                        # if Rover.send_pickup is True:
                        #     Rover.send_pickup = False
                    elif rock_distance < 30:
                        if Rover.vel > 1:
                            Rover.brake = rock_distance/60
                        else:
                            Rover.brake = 0
                        Rover.throttle = Rover.throttle_set
                        Rover.steer = rock_angle*2

                        print("Sample in proximity thr, st, Br, near_sample: ",Rover.throttle,Rover.steer,Rover.brake, Rover.near_sample)
                # elif len(Rover.nav_angles) < Rover.stop_forward:
                #     Rover.mode = 'stop'
                #     return Rover
                if Rover.stuck_frames > 300:
                    Rover.mode = 'stop'
                    Rover.steer = -15*Rover.drive_tendancy
                    Rover.brake = 5
            else:
                if Rover.sample_persistance < 1:
                     Rover.sample_on_sight = False
                     Rover.mode = 'forward'

                Rover.throttle = 0
    # Just to make the rover do something
    # even if no modifications have been made to the code
    elif Rover.nav_angles is None:
        Rover.mode = 'stop'
        Rover.brake = 0
        Rover.throttle = 0
        Rover.steer = -15

    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        Rover.mode = 'reverse'
    print(Rover.mode,Rover.stuck_frames, np.mean(Rover.nav_dists), np.mean(Rover.obstacle_angles* 180 / np.pi),np.min(Rover.obstacle_angles* 180 / np.pi),np.max(Rover.obstacle_angles* 180 / np.pi))
    return Rover
