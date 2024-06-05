/*
  EEZYbotCONTROL.ino - Library for controlling the EEZYbotARM Mk2.
  Created by João Coelho, February 20th, 2024.
  Released into the public domain.
*/

#define hip_pin 27
#define shoulder_pin 16
#define elbow_pin 17
#define gripper_pin 26

const float rad = 0.01745329251;
const float deg = 57.2957795131;

Servo hip, shoulder, elbow;

float last_pos[3];

/*
  Function:   Start
  -----------------
  Initializes the servos and drives the robot to the home position.
*/

void Start()
{
  hip.attach(hip_pin);
  shoulder.attach(shoulder_pin);
  elbow.attach(elbow_pin);

  pinMode(gripper_pin, OUTPUT);
  digitalWrite(gripper_pin, LOW);

  hip.setPeriodHertz(50);
  shoulder.setPeriodHertz(50);
  elbow.setPeriodHertz(50);

  // drive and save home position
  hip.write(90);
  shoulder.write(90);
  elbow.write(0);
  FK(new float[3]{ 90 * rad, 90 * rad, 0 * rad }, last_pos);
}

/*
  Function:   FK
  --------------
  Computes the forward kinematics of a 3DOF robot (EEZYbotARM Mk2).

  Parameters:
	ang:   Array containing the joint angles in radians.
	pos:   Array to store the calculated position of the end effector.

  Note:
	The 'pos' array will be populated with the calculated position coordinates.
*/

void FK(const float *ang, float *pos)
{
  // length of the links
  const float d1 = 0.095, d2 = 0.135, d3 = 0.147;

  float t1 = ang[0];
  float t2 = ang[1];
  float t3 = ang[2];

  // adjust gear ratio (reduction)
  t1 = PI / 2 + ((t1 - PI / 2) / 2);

  // invert rotation of the elbow back to its conventional frame
  t3 = -t3;

  // calculation of the position
  float x = d3 * cos((4 * PI) / 9 - t3) * cos(PI / 2 - t1) * cos(t2) + d3 * sin((4 * PI) / 9 - t3) * cos(PI / 2 - t1) * sin(t2) + d2 * cos(PI / 2 - t1) * cos(t2);
  float y = -d3 * sin((4 * PI) / 9 - t3) * sin(PI / 2 - t1) * sin(t2) - d2 * sin(PI / 2 - t1) * cos(t2) - d3 * cos((4 * PI) / 9 - t3) * sin(PI / 2 - t1) * cos(t2);
  float z = d1 + d2 * sin(t2) + d3 * cos((4 * PI) / 9 - t3) * sin(t2) - d3 * sin((4 * PI) / 9 - t3) * cos(t2);

  // save the positions
  pos[0] = x, pos[1] = y, pos[2] = z;
}

/*
  Function:   IK
  --------------
  Computes the inverse kinematics of a 3DOF robot (EEZYbotARM Mk2).

  Parameters:
	pos:   Array containing the position of the end effector.
	ang:   Array to store the angles of each joint in radians.

  Note:
	The 'ang' array will be populated with the calculated joint angles.
*/

void IK(const float *pos, float *ang)
{
  // length of the links
  const float d1 = 0.095, d2 = 0.135, d3 = 0.147;

  float x = pos[0];
  float y = pos[1];
  float z = pos[2];

  // calculation of the angles
  float t1 = atan2(y, x);
  float t3 = -acos((x * x + y * y + (z - d1) * (z - d1) - d2 * d2 - d3 * d3) / (2 * d2 * d3));
  float t2 = atan2((z - d1), (sqrt(x * x + y * y))) - atan2((sin(t3) * d3), (d2 + cos(t3) * d3));

  t1 = t1 + PI / 2;             // offset of the hip servo
  t3 = (t3 + 4 * PI / 9) * -1;  // offset of the elbow servo and rotation of the frame (the servo rotates counter-clockwise in this frame)

  // adjust gear ratio (multiplication)
  t1 = PI / 2 + ((t1 - PI / 2) * 2);

  // save the angles
  ang[0] = t1, ang[1] = t2, ang[2] = t3;
}

/*
  Function:   LinearInterpolation
  -------------------------------
  Perform linear interpolation between two points in a 3D space.

  Parameters:
	current_pos:   Array containing the current position of the end effector in Cartesian space.
	wanted_pos:    Array containing the desired position of the end effector in Cartesian space.
	velocity:      The velocity at which the interpolation is performed.
	num_steps:     Reference parameter to store the number of steps.
	time_step:     Reference parameter to store the time duration for each step.

  Returns:
	Two-dimensional array with the interpolated positions between the current position and the wanted position.
*/

float **LinearInterpolation(const float *current_pos, const float *wanted_pos, float velocity, int *num_steps, float *time_step)
{
  // calculate the total distance each servo needs to move
  const float dx = wanted_pos[0] - current_pos[0];           // calculate the distance in the 'x' direction
  const float dy = wanted_pos[1] - current_pos[1];           // calculate the distance in the 'y' direction
  const float dz = wanted_pos[2] - current_pos[2];           // calculate the distance in the 'z' direction
  const float distance = sqrt(dx * dx + dy * dy + dz * dz);  // calculate the total distance

  const float resolution = 0.001;                                            // resolution of the step
  velocity = (velocity > resolution / 0.02) ? resolution / 0.02 : velocity;  // limits the velocity
  *num_steps = ceil(distance / resolution);                                  // calculation of number of steps
  *time_step = (distance / velocity) / *num_steps;                           // calculate the time for each step

  // allocate 2D array to save the interpolated points
  float **lerp_matrix = (float **)malloc(*num_steps * sizeof(float *));
  for (int i = 0; i < *num_steps; i++)
  {
    lerp_matrix[i] = (float *)malloc(3 * sizeof(float));
  }

  // calculate the interpolated points
  for (int i = 0; i < *num_steps; i++)
  {
    const float t = distance * (i + 1) / *num_steps;
    lerp_matrix[i][0] = current_pos[0] + (t / distance) * dx;
    lerp_matrix[i][1] = current_pos[1] + (t / distance) * dy;
    lerp_matrix[i][2] = current_pos[2] + (t / distance) * dz;
  }

  return lerp_matrix;
}

/*
  Function:   JointInterpolation
  ------------------------------
  Perform joint interpolation between two points in a 3D space.

  Parameters:
	current_pos:   Array containing the current position of the end effector in Cartesian space.
	wanted_pos:    Array containing the desired position of the end effector in Cartesian space.
	velocity:      The velocity at which the interpolation is performed.
	num_steps:     Reference parameter to store the number of steps.
	time_step:     Reference parameter to store the time duration for each step.

  Returns:
	Two-dimensional array with the interpolated positions between the current position and the wanted position.
*/

float **JointInterpolation(const float *current_pos, const float *wanted_pos, float velocity, int *num_steps, float *time_step)
{
  float current_ang[3], wanted_ang[3];

  IK(current_pos, current_ang);
  IK(wanted_pos, wanted_ang);

  // calculate the total distance each servo needs to move
  const float dx = wanted_pos[0] - current_pos[0];           // calculate the distance in the 'x' direction
  const float dy = wanted_pos[1] - current_pos[1];           // calculate the distance in the 'y' direction
  const float dz = wanted_pos[2] - current_pos[2];           // calculate the distance in the 'z' direction
  const float distance = sqrt(dx * dx + dy * dy + dz * dz);  // calculate the total distance

  const float resolution = 0.001;                                            // resolution of the step
  velocity = (velocity > resolution / 0.02) ? resolution / 0.02 : velocity;  // limits the velocity
  *num_steps = ceil(distance / resolution);                                  // calculation of number of steps
  *time_step = (distance / velocity) / *num_steps;                           // calculate the time for each step

  // allocate 2D array to save the interpolated points
  float **joint_matrix = (float **)malloc(*num_steps * sizeof(float *));
  for (int i = 0; i < *num_steps; i++)
  {
    joint_matrix[i] = (float *)malloc(3 * sizeof(float));
  }

  // calculate the incremental values
  float delta_ang[3];
  delta_ang[0] = (wanted_ang[0] - current_ang[0]) / *num_steps;
  delta_ang[1] = (wanted_ang[1] - current_ang[1]) / *num_steps;
  delta_ang[2] = (wanted_ang[2] - current_ang[2]) / *num_steps;

  // update current angles and populate joint_matrix
  for (int i = 0; i < *num_steps; i++)
  {
    joint_matrix[i][0] = current_ang[0] + delta_ang[0] * (i + 1);
    joint_matrix[i][1] = current_ang[1] + delta_ang[1] * (i + 1);
    joint_matrix[i][2] = current_ang[2] + delta_ang[2] * (i + 1);
  }

  return joint_matrix;
}

/*
  Function:   MoveL
  -----------------
  Perform a linear movement of a 3DOF robot (EEZYbotARM Mk2).

  Parameters:
	pos:        The desired end effector position.
	velocity:   The velocity at which the robot moves to complete the trajectory.
*/

void MoveL(const float *pos, const float velocity)
{
  float *current_pos = nullptr;
  if (xSemaphoreTake(last_pos_mutex, portMAX_DELAY) == pdTRUE)
  {
    current_pos = GetLastPosition();
    xSemaphoreGive(last_pos_mutex);
  }
  const float wanted_pos[3] = { pos[0], pos[1], pos[2] };

  // linearly interpolate between the current position and the wanted position
  int num_steps;                                                                                         // value of the number of steps
  float time_step;                                                                                       // value of the time between each step
  float **lerp_matrix = LinearInterpolation(current_pos, wanted_pos, velocity, &num_steps, &time_step);  // calculate the interpolated positions

  // calculate the inverse kinematics of each position and write to the servo motors
  for (int i = 0; i < num_steps; i++)
  {
    // calculate the inverse kinematics
    float lerp_angles[3];
    IK(lerp_matrix[i], lerp_angles);

    WriteToPhysicalMotors(lerp_angles);

    // save last position
    if (xSemaphoreTake(last_pos_mutex, portMAX_DELAY) == pdTRUE)
    {
      SetLastPosition(lerp_matrix[i]);
      xSemaphoreGive(last_pos_mutex);
    }

    delay(time_step * 1000);
  }

  // free memory of 2D array
  for (int i = 0; i < num_steps; i++)
  {
    free(lerp_matrix[i]);
  }
  free(lerp_matrix);
}

/*
  Function:   MoveJ
  -----------------
  Joint movement of a 3DOF robot (EEZYbotARM Mk2).

  Parameters:
	pos:        The desired end effector position.
	velocity:   The velocity at which the robot moves to complete the trajectory.
*/

void MoveJ(const float *pos, const float velocity)
{
  float *current_pos = nullptr;
  if (xSemaphoreTake(last_pos_mutex, portMAX_DELAY) == pdTRUE)
  {
    current_pos = GetLastPosition();
    xSemaphoreGive(last_pos_mutex);
  }

  const float wanted_pos[3] = { pos[0], pos[1], pos[2] };

  // calculate step factors for each servo
  int num_steps;                                                                                         // value of the number of steps
  float time_step;                                                                                       // value of the time between each step
  float **joint_matrix = JointInterpolation(current_pos, wanted_pos, velocity, &num_steps, &time_step);  // calculate the interpolated positions

  // incrementally move each servo to its wanted position
  for (int i = 0; i < num_steps; i++)
  {
    WriteToPhysicalMotors(new float[3]{ joint_matrix[i][0], joint_matrix[i][1], joint_matrix[i][2] });

    // save updated position
    float updated_pos[3];
    FK(joint_matrix[i], updated_pos);

    if (xSemaphoreTake(last_pos_mutex, portMAX_DELAY) == pdTRUE)
    {
      SetLastPosition(updated_pos);
      xSemaphoreGive(last_pos_mutex);
    }

    delay(time_step * 1000);
  }

  // free memory of 2D array
  for (int i = 0; i < num_steps; i++)
  {
    free(joint_matrix[i]);
  }
  free(joint_matrix);
}

/*
  Function:   WriteToPhysicalMotors
  ---------------------------------
  Write the angles to the physical servo-motors of the robot (EEZYbotARM Mk2).

  Parameters:
	ang:   Array containing the joint angles in radians.
*/

void WriteToPhysicalMotors(const float *ang)
{
  hip.write(ang[0] * deg);
  shoulder.write(ang[1] * deg);
  elbow.write(ang[2] * deg + 90 - ang[1] * deg);
}

/*
  Function:   ToggleGripper
  -------------------------
  Toggle the state of the gripper of the robot.

  Parameters:
	state:   Boolean value indicating whether to open (true) or close (false) the gripper.
*/

void ToggleGripper(const bool state)
{
  digitalWrite(gripper_pin, state);
}

/*
  Function:   WaitTime
  --------------------
  Delays the program execution for a specified amount of time.

  Parameters:
	time:   The duration to wait in seconds.
*/

void WaitTime(const int time)
{
  delay(time * 1000);
}

/*
  Function:   UpdateRobotPosition
  -------------------------------
  Update the robot position based.
*/

void UpdateRobotPosition(const float *pos)
{
  float ang[3];
  IK(pos, ang);
  WriteToPhysicalMotors(ang);
  if (xSemaphoreTake(last_pos_mutex, portMAX_DELAY) == pdTRUE)
  {
    SetLastPosition(pos);
    xSemaphoreGive(last_pos_mutex);
  }
}

/*
  Function:   SetLastPosition
  ---------------------------
  Saves an updated position to "last_pos".

  Parameters:
	pos:   Array containing the last position of the end effector.
*/

void SetLastPosition(const float *pos)
{
  last_pos[0] = pos[0];
  last_pos[1] = pos[1];
  last_pos[2] = pos[2];
}

/*
  Function:   GetLastPosition
  ---------------------------
  Returns the position saved on "last_pos".

  Returns:
	Array containing the last position of the end effector.
*/

float *GetLastPosition()
{
  return last_pos;
}
