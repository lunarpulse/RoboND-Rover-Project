## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

*   Download the simulator and take data in "Training Mode"
*   Test out the functions in the Jupyter Notebook provided
*   Add functions to detect obstacles and samples of interest (golden rocks)
*   Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map. The `output_image` you create in this step should demonstrate that your mapping pipeline works.
*   Use `moviepy` to process the images in your saved dataset with the `process_image()` function. Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

*   Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook).
*   Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands.
*   Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[image0]: ./misc/screenshottesting.png
[image1]: ./misc/rover_image.jpg
[image4]: ./misc/calibration.png
[image3]: ./misc/obstacles.png
[image2]: ./misc/filteredImages.png

[video1]: ./output/4rocksHighfidelity.avi

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. This README file is to discuss about how this project was solved and more possibility of better solutions to this specific sample collection and homing objectives.

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
The note book was modified to test the perception part of the project.
Images of rock sample were taken from the training run and it was used to find the thresh hold for the yellow sample. After experimentation, I actually found some hint from the threads from slack. The image below shows how selectively filters out the rock sample from the environment.

I found that the recognition of the yellow sample in the shade, especially in a few shady places in the map, were not well recognised. The overall performance of rock sample recognition was good in the autonomous run in later this project.

The other part experimented in the jupyter notebook was to experimenting the cutoff area to filterout the sky. This was to reduce errors in mapping and also improve the rover stablity, ported in perception_step function later. The sky colour was similar to the ground and when rover slams the brake for any reason, the unstability of perception introduced few unexpected behaivours of robot. Therefore, the 5/12 from the top part of the image was cut prior to the perspective transformation.
This ensures stability and robustness of the perception, which lead to 95% to 100% match rate with the ground truth map with low speed roaming, less than 0.5m/s, during the autonomous mode.

The last experimentation in the notebook was cv2 filtering to smooth the navigable and obstacle parts. Both were achieved with polymorphic transformation yet different filtering methods in the obstacle thresh and colour thresh function.
a dilation and erosion filter with different filter sizes was used in the olour thresh and the obstacle thresh function respectively. The rock samle filter was unfiltered except the cropping in before the persepective transformation, before feeding into colour thresh hold functions.

After all the initial set up, the test run in the simulator was converted into a short video clip showing how the perception of obstacles, navigable zones and rock samples. It is included in the jupyter notbook,Rover_project_RockSample.ipynb and below image.

By completing this, perception part of the project was almost done, it only required some fine tuning to get high fidelity during the run.


![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap. Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.
The lecture notes were directly referred to complete a few blank in the functions in perception.py. Setting up the processing_image was similar to the jupyter notebook solution.

There is a request or todo, I want the log file also dump keyboard commands too. This can make the dataset to be used for training neural networks and all those computer vision code can be trained. In other words, recognition of fetures like yellow sample rocks and walls and navigable roads can be more robust and related parameters can be establiished more flexibly and in a robust manner.

Some extra test for openCV was done in [opencv2cvt.ipynb](./code/opencv2cvt.ipynb).
![alt text][image2]
Figure above shows the masked images during the process_image. The first and second from the right shows the navigable and obstacles respectively. The third and fourth shows the filtered image though rock sample threshold function.

![alt text][image3]
Obstacles had been processed with more filters as poorly filetered obstacles seem to be the cause of fidelity decrease. Thin and accurate boundary of the obstacles were drawn with dilation from open cv library and bitwise_and with masked navigable zone.

```python
def obstacle_filter(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select_navi = np.zeros_like(img[:,:,0])
    wall= np.zeros_like(img[:,:,0])
    color_select_obst = np.zeros_like(img[:,:,0])
    obs= np.zeros_like(img[:,:,0])

    kernel = np.ones((5,5),np.uint8)

    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    color_select_navi[above_thresh] = 1
    #Morphological filter
    dilation = cv2.dilate(color_select_navi,kernel,iterations = 1)
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
```

The picture above shows how a narrow road is processed with the function.

![alt text][image4]
The image above shows the result of calibration. The matching rate of the ground truth map and the result is very high. only sudden turn or roll and pitch higher than 1 degree or lower than 359 causes invalid mapping. This leads a high fidelity, more than 80%, in autonomous runs. This [test run video](./output/test_run_dataset.mp4) and [rock picking video](./output/test_mapping_rock_pick.mp4) contains the entire footage of the test run and performance of the functions written.

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.
rock sample chasing at first
1.running away from the sensor because of the unstable continuinity of sensor recognition.
once the paramaters are calibrated, it chases once any rock sample is in sight. However, sometimes it stuck to the rough edge of the terrain. A unstuck code was implemented by wiggling its steering a little and reverse a certain distance.

making the rover following the right wall was a tricky job. My current code depends heavily on the speed when the mean angle of the object angles are outside of set range. This thresh hold was found in numerous iteration of run.
Using mean angle deviation was a simple way of doing this, but it does not provide wall following all the time. A new way of path planning using the world map is required to tackle this issue.
I am working on this thought now.

The code written has 4 different modes while any navigable area exists during the perception_step, **forward, stop, apprach, reverse.** The other high tier is return to home branch once **finished** is set to true, yet experimental at the moment, works partially not yet tuned satisfactory.

With a number of thresh holds, obtained through the fine tuning during the tests, the mode change happens and reacts with the environmental factors, mainly the navigable and obstacle distance and angles.


In addition, the ***stuck_frames*** parameter, counting for near zero velocity for how many frames, and this triggers some unstuck codes comprising of reverse with certain angles or high throttle forward with an angle.

The ***Rover.sample_on_sight*** property was used to focus on the sample and trigger approach mode and the rover persues the rock on the screen. However, the rover sometimes get stuck after passing the sample in some cases, **stuck_frames** more than a set number sets mode of rover to forward or reverse to try to find the nearby sample just passed again.

Going home function works only if the rover is near the centre area at the moment. If the rover is far from centre, it struggles a little. These going home function was implemented by calculating yaw angle to the stored starting position and turning the rover to this point until current yaw points to this point within a set thresh hold. Afterwards, similar to normal collision avoidance and correcting its yaw to the position.
One of the difficulty to get the *finished* property set to true was the python code was rather multi threaded so that the code had to have a timer to count the time between the sample pickup and decide whether it is a genuine pick up or just duplicated one from other threads.

It is slow and return time depends on where rover roams. This can be replaced with A* algorithm with heuristic array defined to the starting position. I am working on it too.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously. Explain your results and how you might improve them in your writeup.  

[autonomous run in high fidelity](./output/4rocksHighfidelity.avi)
This video via the link shows how the implementation in decision_step and perception_step works. It works nicely most time. The resolution of video is 768 by 480 for 25 fps recordred by vokoscreen. Due to the limited computing power for the demanding process, Rover_sim and the python codes, it was inevitable to downgrade the screen resolution. With screen recording the fps of the perception step goes down even further and such slew impacted the overal autonomous performance. I wish I could make more processing efficient codes. I guess the open cv filtering and other heavy third party based functions are the ones draining most of the frames per seconds performance.
![testing in progress][image0]

Overall, this set of code works well in this environment. By 30 times of autonomous run, it picks up more than 4 rocks average as long as it does not get stuck between a big rocks where the polygon becomes transparent when the rover is nearby. However, there are some pitfalls here. If the shape of terrain changes, it might not work well as this code is tuned to work in this terrain. At the moment, my code does not plan the route more likely react to perception. When it sees a yellow rock, it is attracted to the rock and loses its track on wall following. Memorising where it visited and planning with it can be an approach. Solutions for various situations had been devised, including stuck in the quick sand near black boulders, bypassing cluttered middle starting point, slowing down near the samples. Most of the codes were written on the go, instead of make a grand plan for it. Still I feel there are rooms to improve and not totally happy with this quick job. In my head, some thoughts about path planning based on proper artificial intellegence learnt from artificial intellegence for robotics course by Prof. Sebastian Thurn at udacity, and also neural networks and machine learning of the current hot trends. I want to implment [A*](https://classroom.udacity.com/courses/cs373/lessons/48646841/concepts/487510240923) or [dynamic programming](https://classroom.udacity.com/courses/cs373/lessons/48646841/concepts/486679840923) for homing and also general navigating functions for this project. In addition, I want to make this more for machine learning project like this [auto RC car](https://github.com/hamuchiwa/AutoRCCar.git) as this project is similar to auto RC car project.

Futrhermore, I would like to use the created world map more extensively than the current set up. As using the created map with high fidelity is more elegant solution where roboticists are heading towards.
