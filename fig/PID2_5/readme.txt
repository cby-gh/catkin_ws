I check the details of the performance of PID controller and question1 for details and make sure that there exist a gap between the drake and gazebo, so it need to pay more attention when make drake-->gazebo(ros)-->physical robot(real world)!
And I assign a effort PID controller to theta joint explicitly and contrain the effort to 0, so that theta joint still a passive joint. 
