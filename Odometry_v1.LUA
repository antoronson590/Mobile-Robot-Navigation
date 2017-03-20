-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

-- Angle Correction (V-REP -> Real System)
function angle_correction(angle)
  -- The allowed angle of rotation are 0~360. However, it is observed, the rotation is moving beyond 360.
  local new_angle = math.abs(angle*180/math.pi) - 360;
  new_angle = new_angle*(math.pi/180) * (angle/math.abs(angle));
  return new_angle
end

-- To set wheel vlocity, based on sensor reading
function wheel_velocity(input_handle, input_value,l_joint, r_joint)
  local value = (simGetUISlider(input_handle,input_value))-500	-- Provision to adjust the max wheel velocity, for turning
  local velocity = -10;											-- Max Wheel Velocity(in deg/s)
  if value<0 then												-- Left Turn
    vel_l = velocity*(500-math.abs(value))/500;
    simSetJointTargetVelocity(l_joint,vel_l);
  else
    simSetJointTargetVelocity(l_joint,velocity);
  end
  if value>0 then												-- Right Turn
    vel_r = velocity * (500-value)/500;
    simSetJointTargetVelocity(r_joint,vel_r);
  else
    simSetJointTargetVelocity(r_joint,velocity);
  end
end

--  To determine the distance wheel has moved for every time step
function delta_x(joint)
  -- We calculate how much distance have it moved between the last and current position of joint
  d_theta = l_per_deg         -- Calculate from the wheel circumference.
  d_thet = simGetJointTargetVelocity(joint);
  dt = simGetSimulationTimeStep();
  local distance = (d_thet)*dt*d_theta*180/math.pi;
  return distance;
end

-- Returns the parameters [new positon x, new position y, new orientation theta]
function delta_odometry(x_0,y_0,alpha_0,dx,dy)
  -- The formulas are absorbed from the research paper "A Primer on Odometry and Motion Control"
  local d_baseline = 0.116               -- The distance between wheel center
  local dcenter = (dx+dy)/2;
  local d_phi = (dy-dx)/(d_baseline);
  local alpha = alpha_0+d_phi;
  if((math.abs(alpha*180/math.pi))>360) then    -- When angle crosses 2PI
    alpha = angle_correction(alpha);	-- in rad
  end
  local x = x_0+(dcenter*math.cos(alpha_0));-- in m
  local y = y_0+(dcenter*math.sin(alpha_0));-- in m 
  return x,y,alpha;
end

-- Initialization block ( Variable initializaiton and component initiailziation
if (sim_call_type==sim_childscriptcall_initialization) then

  -- Slider Initialization
  input_handle = simGetUIHandle('Input_Slider');
  input_value = 3;
  -- Graph Handle
  graph = simGetObjectHandle('Graph');
  graph_alp = simGetObjectHandle('Graph0');
  -- Circumference calculation(Constant)
  dia = 0.1;					-- Wheel Diameter[m]
  l_per_deg = math.pi*dia/360;	-- circum/deg [m/deg]

  -- Joint initialization and initial joint position determination
  l_joint = simGetObjectHandle('Joint_Left');
  r_joint = simGetObjectHandle('Joint_Right');
  l_pos_old = simGetJointPosition(l_joint);
  r_pos_old = simGetJointPosition(r_joint);
  l_pos = 0;
  r_pos = 0;


  -- Variable Initialization
  alph_0 = 0;  -- Initial Orientation of Body
  x_0 = 0       -- Initial Position - x
  y_0 = 0       -- Initial Position - y
  x = 0         -- Initialization
  y = 0;
  del_l = 0;    -- Change in left Wheel
  del_r = 0;    -- Change in right Wheel
  alph  = 0;    -- Orientation Initialization

  -- Error Plotting
  position = {};
  body = simGetObjectHandle('Body');
  err_x = 0;
  err_y = 0;
  err = simGetObjectHandle('Graph_Error');
  agraph = simGetObjectHandle('Graph_Actual');

  -- Sensor Initialization and Reading
  sensor = simGetObjectHandle('Proximity_sensor');
  s_value = 0;

  -- Method 2
  x_m2 = 0
  y_m2 = 0
  theta_m2 = 0;
  x_m2_0 = 0
  y_m2_0 = 0
  theta_m2_0 = 0
  orientat = {};
  orientat = simGetObjectOrientation(body,-1);
  theta_m2_0 = orientat[3];
  mat = {};
  mat_old = simGetObjectMatrix(body,-1);


end

-- Executed for the rest of the simulation
if (sim_call_type==sim_childscriptcall_actuation) then
  s_value= simReadProximitySensor(sensor);		-- Sensor Reading[1 -> Detected Blocking, -1 -> Error, 0 -> Free]
  if s_value == 1 then
    simSetUISlider(input_handle,input_value,100);
  else
    simSetUISlider(input_handle,input_value,500);
  end    
  
  wheel_velocity(input_handle, input_value,l_joint, r_joint)	-- Set Wheel Velocity
  
  l_pos = simGetJointPosition(l_joint);							-- Change on joint position (left & right)
  r_pos = simGetJointPosition(r_joint);
  
  del_l = delta_x(l_joint);										-- Distance moved (left & right)
  del_r = delta_x(r_joint);
  

  x,y,alph = delta_odometry(x_0,y_0,theta_m2_0,del_l,del_r)		-- Position determination

  simSetGraphUserData(agraph,'x_value',-x);
  simSetGraphUserData(agraph,'y_value',y);
--  simSetGraphUserData(agraph,'x_value_m',-x_m2);
--  simSetGraphUserData(agraph,'y_value_m',y_m2);
--  simSetGraphUserData(graph_alp,'Angle',alph*180/math.pi);
  position = simGetObjectPosition(body, -1);
  err_x = position[1] - x; 
  err_y = position[2] - y;
--  simSetGraphUserData(err, 'Error_x',(err_x));
--  simSetGraphUserData(err, 'Error_y',(err_y));
   
  -- Data Exchange
  l_pos_old = l_pos;
  r_pos_old = r_pos;
  x_0 = x;
  y_0 = y;
  alph_0 = alph;

end


if (sim_call_type==sim_childscriptcall_sensing) then

	-- Put your main SENSING code here

end


if (sim_call_type==sim_childscriptcall_cleanup) then

	-- Put some restoration code here

end-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)
-- Angle Correction

