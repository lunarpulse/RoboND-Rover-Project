## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map. The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function. Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook).
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands.
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
The note book was modified to test the perception part of the project.
images of rock sample were taken from the training run and it was used to find the thresh hold for the yellow sample. After experimentation, I actually found some hint from the threads from slack. The image below shows how selectively filters out the rock sample from the environment.

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

![alt text][image2]
### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.
rock sample chasing at first
1.running away from the sensor because of the unstable continuinity of sensor recognition.
once the paramaters are calibrated, it chases once any rock sample is in sight. However, sometimes it stuck to the rough edge of the terrain. A unstuck code was implemented by wiggling its steering a little and reverse a certain distance.

making the rover following the right wall was a tricky job. My current code depends heavily on the speed when the mean angle of the object angles are outside of set range. This thresh hold was found in numerous iteration of run.
Using mean angle deviation was a simple way of doing this, but it does not provide wall following all the time. A new way of path planning using the world map is required to tackle this issue.
I am working on this thought now.

going home function works if the rover is near the centre area. If the rover is far from centre, it struggles a little. These going home function was implemented by calculating yaw angle to the stored starting position and turning the rover to this point until current yaw points to this point within a set thresh hold. Afterwards, similar to normal collision avoidance and correcting its yaw to the position.
It is slow and return time depends on where rover roams. This can be replaced with A* algorithm with heuristic array defined to the starting position. I am working on it too.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously. Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines! Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  



![alt text][image3]
