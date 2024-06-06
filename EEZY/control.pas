{
  Control.pas - Simulation of the EEZYbotARM Mk2.
  Created by João Coelho, March 11st, 2024.
  Released into the public domain.
}

type
  SingleArray = array[0..2] of Single;
  
var
  mode: String;
  last_pos: SingleArray;
  program_folder: TFileStream;
  point_list: TStringList;

// reference functions and procedures
function ExtractDelimited(str: String; delimiter: Char; index: Integer): String; forward;
function FK(ang: SingleArray): SingleArray; forward;
function IK(pos: SingleArray): SingleArray; forward;
function IsWithinWorkspace(pos: SingleArray): Boolean; forward;
procedure WriteToSimulatedMotors(pos: SingleArray); forward;
procedure SavePointsForPhysicalRobot; forward;
procedure SendRobotPosition(pos: SingleArray); forward;
procedure ReceiveRobotPosition; forward;
procedure ReadChangeModeButton; forward;
procedure ReadControlButtons; forward;
procedure ReadCreatePointButton; forward;
procedure WriteToSheet; forward;

{
  function:   ExtractDelimited
  ----------------------------
  Extracts a delimited substring from a string based on the specified index.

  Parameters:
    str:         String containing the source string.
    delimiter:   Char representing the delimiter character.
    index:       Integer indicating the index of the delimited substring to extract.

  Returns:
    Extracted delimited substring.
}

function ExtractDelimited(str: String; delimiter: Char; index: Integer): String;
var
  start_pos, end_pos, count: Integer;
begin
  Result := '';
  count := 0;
  start_pos := 1;

  // find the start position of the specified index
  while (count < index) and (start_pos <= Length(str)) do
  begin
    if str[start_pos] = delimiter then
      Inc(count);
    Inc(start_pos);
  end;

  // find the end position of the specified index
  end_pos := start_pos;
  while (end_pos <= Length(str)) and (str[end_pos] <> delimiter) do
    Inc(end_pos);

  // extract the substring between start and end positions
  Result := Copy(str, start_pos, end_pos - start_pos);
end;

{
  Function:   FK
  --------------
  Computes the forward kinematics of a 3DOF robot (EEZYbotARM Mk2).

  Parameters:
    ang:   Array with the angles of each joint.

  Returns:
    An array containing the position of the end effector.
}

function FK(ang: SingleArray): SingleArray;
var
  d1, d2, d3: Single;
  t1, t2, t3: Single;
  x, y, z: Single;
begin
  // initialize the values of d1, d2, and d3
  d1 := 0.095;
  d2 := 0.135;
  d3 := 0.147;

  t1 := ang[0];
  t2 := ang[1];
  t3 := ang[2];

  // adjust gear ratio (reduction)
  t1 := Pi/2 + ((t1 - Pi/2) / 2);

  // calculate the position of the end effector
  x := d3 * cos((4 * Pi) / 9 - t3) * cos(Pi / 2 - t1) * cos(t2) +
       d3 * sin((4 * Pi) / 9 - t3) * cos(Pi / 2 - t1) * sin(t2) +
       d2 * cos(Pi / 2 - t1) * cos(t2);                           // position 'x' of the end effector
  y := -d3 * sin((4 * Pi) / 9 - t3) * sin(Pi / 2 - t1) * sin(t2) -
       d2 * sin(Pi / 2 - t1) * cos(t2) -
       d3 * cos((4 * Pi) / 9 - t3) * sin(Pi / 2 - t1) * cos(t2);  // position 'y' of the end effector
  z := d1 + d2 * sin(t2) + d3 * cos((4 * Pi) / 9 - t3) * sin(t2) -
       d3 * sin((4 * Pi) / 9 - t3) * cos(t2);                     // position 'z' of the end effector

  // return the positions as an array
  Result[0] := x;                                                 // assign the 'x' coordenate
  Result[1] := y;                                                 // assign the 'y' coordenate
  Result[2] := z;                                                 // assign the 'z' coordenate
end;

{
  Function:   IK
  --------------
  Computes the inverse kinematics of a 3DOF robot (EEZYbotARM Mk2).

  Parameters:
    pos:   Array with the position of the end effector.

  Returns:
    An array containing the angles of each joint.
}

function IK(pos: SingleArray): SingleArray;
var
  d1, d2, d3: Single;
  t1, t2, t3: Single;
  x, y, z: Single;
begin
  // initialize the values of d1, d2, and d3
  d1 := 0.095;
  d2 := 0.135;
  d3 := 0.147;

  x := pos[0];
  y := pos[1];
  z := pos[2];

  // calculate the angles
  t1 := ATan2(y, x);              // calculation of the hip angle
  t3 := -arccos((x * x + y * y + (z - d1) * (z - d1) - d2 * d2 - d3 * d3) /
        (2 * d2 * d3));           // calculation of the elbow angle
  t2 := ATan2((z - d1), (sqrt(x * x + y * y))) -
        ATan2((sin(t3) * d3),
        (d2 + cos(t3) * d3));     // calculation of the shoulder angle

  // fix the angles
  t1 := t1 + Pi / 2;              // rotation offset of the hip servo
  t3 := (t3 + 4 * Pi / 9);        // rotation offset of the elbow servo

  // adjust gear ratio (multiplication)
  t1 := Pi/2 + ((t1 - Pi/2) * 2);

  // return the angles as an array
  Result[0] := t1;                // assign the hip angle
  Result[1] := t2;                // assign the shoulder angle
  Result[2] := t3;                // assign the elbow angle
end;

{
  function:   IsWithinWorkspace
  -----------------------------
  Checks if the wanted point falls inside the robot workspace.
  Note: This is a 1/4 sphere, not representative of the actual robot workspace.

  Parameters:
    x:   The x-coordinate of the end effector.
    y:   The y-coordinate of the end effector.
    z:   The z-coordinate of the end effector.
}

function IsWithinWorkspace(pos: SingleArray): Boolean;
var
  base_rotation, dist_point: Single;
  x, y, z: Single;
begin
  x := pos[0];
  y := pos[1];
  z := pos[2];

  // calculate the value of the base rotation
  base_rotation := ATan2(y, x) + Pi/2;

  // remove base height
  z := z - 0.095;
  // calculate the distance between the point and the base
  dist_point := sqrt(x * x + y * y + z * z);

  // check if the point falls within the defined boundaries
  if (base_rotation >= 0) and (base_rotation <= Pi)  and (dist_point < 0.282) then
  begin
    Result := True;
    exit;
  end;

  Result := False;
end;

{
  procedure:   WriteToSimulatedMotors
  -----------------------------------
  Write the angles to the servo motors inside the simulator.

  Parameters:
    pos:   Array with the position of the end effector.
}

procedure WriteToSimulatedMotors(pos: SingleArray);
var
  ang: SingleArray;
  i: Integer;
begin
  if (IsWithinWorkspace(pos) = false) then exit;

  ang := IK(pos);

  // apply gear ratio (reduction)
  ang[0] := Pi/2 + ((ang[0] - Pi/2) / 2);

  // shoulder and elbow angles rotate counter-clockwise in the simulation
  ang[1] := - ang[1];
  ang[2] := - ang[2];

  // writes to the main robot servo motors
  for i := 0 to 2 do
    SetAxisPosRef(0, i, ang[i]);

  // writes the angle of the gripper (it is always parallel to the base)
  SetAxisPosRef(0, 3, - (ang[1] + ang[2]));

  last_pos := pos;
end;

{
  procedure:   SavePointsForPhysicalRobot
  ---------------------------------------
  Creates a '.h' file with all saved points on the sheet.
}

procedure SavePointsForPhysicalRobot;
var
  point_format: String;
  L: Integer;
begin
  if not RCButtonPressed(7, 2) then exit;

  // initialize the list
  point_list := TStringList.Create;

  // include the headers
  point_list.add('#include <Arduino.h>' + #13#10 + '#include "EEZYbotCONTROL.h"' + #13#10);

  L := 9;
  while (GetRCText(L, 1) <> '') and
        (GetRCText(L, 2) <> '') and
        (GetRCText(L, 3) <> '') and
        (GetRCText(L, 4) <> '') do
  begin
    point_format := format('float %s[] = { %f, %f, %f };',
                   [GetRCText(L, 1), GetRCValue(L, 2), GetRCValue(L, 3), GetRCValue(L, 4)]);
    point_list.add(point_format);
    Inc(L);
  end;

  point_list.add(#13#10 + 'void MAIN();');

  point_list.SaveToFile('RUNME\Prog.h');
  point_list.Free;
end;

{
  procedure:   SendRobotPosition
  ------------------------------
  Sends an updated position to the physical robot when manual mode is active.
}

procedure SendRobotPosition(pos: SingleArray);
var
  packet: String;
begin
  packet := format('manual,%f,%f,%f,',[pos[0], pos[1], pos[2]]);

  WriteComPort(packet);
end;

{
  procedure:   ReceiveRobotPosition
  ---------------------------------
  Reads the end effector position from the microcontroler and updates the simulated robot.
}

procedure ReceiveRobotPosition;
var
  packet: String;
  pos: SingleArray;
begin
  packet := ReadComPort;

  if (packet = '') then exit;

  try
    pos[0] := StrToFloat(ExtractDelimited(packet, ',', 1));
    pos[1] := StrToFloat(ExtractDelimited(packet, ',', 2));
    pos[2] := StrToFloat(ExtractDelimited(packet, ',', 3));
  except
    WriteLn('Error converting position to float');
    exit;
  end;

  mode := ExtractDelimited(packet, ',', 0);

  WriteToSimulatedMotors(pos);
end;

{
  procedure:   ReadChangeModeButton
  ---------------------------------
  Reads the change mode button and sends the updated mode to the physical robot.
}

procedure ReadChangeModeButton;
begin
  if (not RCButtonPressed(2, 6)) then exit;

  if (mode = 'automatic') then
    WriteComPort('manual,' + FloatToStr(last_pos[0]) + ',' + FloatToStr(last_pos[1]) + ',' + FloatToStr(last_pos[2]))
  else if (mode = 'manual') then
    WriteComPort('automatic,' + FloatToStr(last_pos[0]) + ',' + FloatToStr(last_pos[1]) + ',' + FloatToStr(last_pos[2]));
end;

{
  procedure:   ReadControlButtons
  -------------------------------
  Reads the control buttons and sends the updated position to the physical robot.
}

procedure ReadControlButtons;
var
  temp_pos: SingleArray;
  step: Single;
  i: Integer;
begin
  if (not (mode = 'manual')) then exit;

  // read the step value
  step := GetRCValue(1, 2);

  // limit the step value
  if (step > 0.05) then
    step := 0.05;

  // save temporary position
  temp_pos := last_pos;

  // send the updated position to the robot based on the buttons pressed
  for i := 2 to 4 do
  begin
    if RCButtonPressed(i, 1) then
    begin
      temp_pos[i - 2] := last_pos[i - 2] - step;
      SendRobotPosition(temp_pos);
      exit;
    end
    else if RCButtonPressed(i, 2) then
    begin
      temp_pos[i - 2] := last_pos[i - 2] + step;
      SendRobotPosition(temp_pos);
      exit;
    end;
  end;
end;

{
  Procedure:   ReadCreatePointButton
  ----------------------------------
  Reads the create button and creates a new instanced point on the points table.
}

procedure ReadCreatePointButton;
var
  L: Integer;
begin
  if not RCButtonPressed(7, 1) then exit;

  L := 9;
  while (GetRCText(L, 2) <> '') and
        (GetRCText(L, 3) <> '') and
        (GetRCText(L, 4) <> '') do
    Inc(L);

  // writes the position in the table
  SetRCValue(L, 2, FloatToStr(last_pos[0]));
  SetRCValue(L, 3, FloatToStr(last_pos[1]));
  SetRCValue(L, 4, FloatToStr(last_pos[2]));
end;

{
  Procedure:   WriteToSheet
  -------------------------
  Writes read-only parameters to the sheet.
}

procedure WriteToSheet;
begin
  // writes the current position of the robot's end effector
  SetRCValue(2, 4, FloatToStr(last_pos[0]));
  SetRCValue(3, 4, FloatToStr(last_pos[1]));
  SetRCValue(4, 4, FloatToStr(last_pos[2]));

  // writes the current operating mode of the robot
  SetRCValue(2, 7, mode);
end;

procedure Control;
begin
  // reads the buttons for controling the robot
  ReadControlButtons;
  // reads the create button to create a new instanced position
  ReadCreatePointButton;
  // reads the mode button to change the mode of the robot
  ReadChangeModeButton;
  // writes read-only parameters to the sheet
  WriteToSheet;
  // receives the updated position from the physical robot
  ReceiveRobotPosition;
  // reads save button to save the points to a file
  SavePointsForPhysicalRobot;
end;

procedure Initialize;
begin
  // drive robot to home position
  last_pos[0] := 0.147;
  last_pos[1] := 0;
  last_pos[2] := 0.2285;

  WriteToSimulatedMotors(last_pos);
end;
