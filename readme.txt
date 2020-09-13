drake_control1.py controlls the /gazebo/set_linkstates directly without controller.
drake_control2.py use PID effort controller.
drake_control2_5.py use PID effort controller and make sure that there exist a gap between the drake and gazebo! pay
	attention to this issue.
drake_control2_6.py implements an elegant publish frequency for different execution time from Drake.
@@@drake_control3.py  use the LQR feedback controller for close loop control(feedback explicitly) 
upub.py publish the joint input u of gazebo.
xstatepub.py publish the state x of gazebo.
question1 gazebo need to change to fit the original frake in python, why?

@@@ means not complete!
