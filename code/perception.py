import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    erosion = cv2.erode(color_select,kernel,iterations = 2)
    # Return the binary image
    return erosion

def sample_thresh(img,rgb_thresh1=(100,100,20),rgb_thresh2=(255,255,30)):
    # color_select = np.zeros_like(img[:,:,0])
    low_yellow = np.array([rgb_thresh1[2],rgb_thresh1[1],rgb_thresh1[0]])
    high_yellow = np.array([rgb_thresh2[2],rgb_thresh2[1],rgb_thresh2[0]])

    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV,3)
    sample_mask = cv2.inRange(img_hsv, low_yellow, high_yellow)

    return sample_mask

def obstacle_filter(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select_navi = np.zeros_like(img[:,:,0])
    wall= np.zeros_like(img[:,:,0])
    color_select_obst = np.zeros_like(img[:,:,0])
    obs= np.zeros_like(img[:,:,0])

    kernel = np.ones((10,10),np.uint8)

    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    color_select_navi[above_thresh] = 1
    #Morphological filter
    dilation = cv2.dilate(color_select_navi,kernel,iterations = 3)
    #only boundary
    wall_selection = (color_select_navi != dilation)
    wall[wall_selection] =1

    height,width = wall.shape

    poly_img = np.zeros((height,width), np.uint8)
    clipper = np.array( [[[156,155],[40,60],[260,60],[165,154]]], dtype=np.int32 )
    #clipper = np.array( [[[157,153],[1,42],[0,42],[0,0],[320,0],[320,43],[319,43],[165,153]]], dtype=np.int32 )
    cv2.fillPoly( poly_img, clipper, 1 )
    masked_data = cv2.bitwise_and(wall, wall, mask=poly_img)

    return masked_data

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot / scale))
    ypix_translated = np.int_(ypos + (ypix_rot / scale))
    # Return the result
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image

    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO:

    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    #only interested in these area to reduce noise from scnene
    # sr_o_intrest = np.float32([[0, Rover.img.shape[1]/2], [Rover.img.shape[0] ,Rover.img.shape[1]/2],[Rover.img.shape[0], Rover.img.shape[1]/2], [Rover.img.shape[0], Rover.img.shape[1]/2]])

    # warped_uncliped = perspect_transform(Rover.img, source, destination)
    # sample_threshed = sample_thresh(warped_uncliped)
    # threshed = color_thresh(warped_uncliped)

    height,width,depth = Rover.img.shape
    rect_img = np.zeros((height,width), np.uint8)
    cv2.rectangle(rect_img, (0 ,np.int(height*5/12)),(np.int(width) ,np.int(height)), 1, thickness=-1)

    masked_image = cv2.bitwise_and(Rover.img, Rover.img, mask=rect_img)
    # 2) Apply perspective transform
    warped_cliped = perspect_transform(masked_image, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

    sample_threshed = sample_thresh(warped_cliped)
    threshed = color_thresh(warped_cliped)

    obs_rect_img = np.zeros((height,width), np.uint8)
    cv2.rectangle(obs_rect_img, (0 ,np.int(height*6/12)),(np.int(width) ,np.int(height)), 1, thickness=-1)

    obs_masked_image = cv2.bitwise_and(Rover.img, Rover.img, mask=obs_rect_img)
    # 2) Apply perspective transform
    obs_warped_cliped = perspect_transform(obs_masked_image, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

    obstacles = obstacle_filter(obs_warped_cliped)


    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacles*255
    Rover.vision_image[:,:,1] = sample_threshed*255
    Rover.vision_image[:,:,2] = threshed*255
    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(threshed)
    xobs, yobs = rover_coords(obstacles)
    xrock, yrock = rover_coords(sample_threshed)

    # 6) Convert rover-centric pixel values to world coordinates
    rover_xpos = Rover.pos[0]
    rover_ypos = Rover.pos[1]
    rover_yaw = Rover.yaw
    scale = 10

    obstacle_x_world, obstacle_y_world = pix_to_world(xobs, yobs, rover_xpos,
                                rover_ypos, rover_yaw,
                                Rover.worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(xrock, yrock, rover_xpos,
                                rover_ypos, rover_yaw,
                                Rover.worldmap.shape[0], scale)
    navigable_x_world, navigable_y_world = pix_to_world(xpix, ypix, rover_xpos,
                                rover_ypos, rover_yaw,
                                Rover.worldmap.shape[0], scale)

    # Rover.samples_pos = (np.mean(rock_x_world) ,np.mean(rock_y_world))
    #print(rock_x_world,rock_y_world)
    #print(len(rock_x_world))

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if (0.7>Rover.roll or Rover.roll>359.3) and (0.7>Rover.pitch or Rover.pitch>359.3):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(xpix, ypix)
    #mean_dir = np.mean(rover_centric_angles)
    Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_angles = rover_centric_angles
    if len(xobs) > 0:
        Rover.obstacle_dists, Rover.obstacle_angles  = to_polar_coords(xobs, yobs)
        #obstacle flag?
    if len(xrock) > 0:
        #stauts updte
        Rover.sample_on_sight = True
        Rover.sample_persistance += 1
        rockdists, rockangles = to_polar_coords(xrock, yrock)
        Rover.sample_dists = rockdists
        Rover.sample_angles = rockangles
        #rock_dist = np.mean(rockdists)
        #rock_dir = np.mean(rockangles)
        # print('rock dist and angle: ',rock_dist, rock_dir)
        # print(rockdists, rockangles)
    else:
        if Rover.sample_persistance > 0:
            Rover.sample_persistance -= 1
        else:
            Rover.sample_persistance = 0


    return Rover
