# Mapping

Input:
	Updates, Per-Call:
		StartPose (Odometry-Message)
		TargetPose (Odometry-Message)
		RealPose (Odometry-Message)
	Request for specific position

Output:
	Complete Map
	Specific cell (+Neighbours?)
	Per-Cell:
		Linear Error
		Angular Error
		Data Age --> contained in quality?
		Data Quality

Calculations:
	compare planned vs. real
		position
		orientation
		Change in speed
		change in orientation

SetUp:
	register with each sphero
		Get current pose
		Get planned next pose
	Start computation after second position