%-----------------------------------------------------------------------
% Crane_PID_GUI.m
% Control GUI for gantry crane system (ENGPHYS 3BB3 Final Design Project)
% Written by Jarod Coppens
% Modified from PID_GUI.m written by Kenrick Chin
% Date: April 2019 
%-----------------------------------------------------------------------
function varargout = PID_GUI(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PID_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @PID_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
%-----------------------------------------------------------------------


% --- Executes just before PID_GUI is made visible.
function PID_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
global comport
global RUN
global NPOINTS
RUN = 0;
NPOINTS = 1;
comport = serial('COM8','BaudRate',115200);
fopen(comport)
% Choose default command line output for PID_GUI
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);    
%-----------------------------------------------------------------------
function PID_GUI_OutputFcn(hObject, eventdata, handles, varargin)
%-----------------------------------------------------------------------
function Quit_Button_Callback(hObject, eventdata, handles)
global comport
global RUN
RUN = 0;
fclose(comport)
clear comport
% use close to terminate your program
% use quit to terminate MATLAB
close
%----------------------------------------------------------------------
function Plot_Button_Callback(hObject, eventdata, handles)
global comport
global NPOINTS
global RUN
global targetposition
% global fid
if RUN == 0
  RUN = 1;
  % change the string on the button to STOP
  set(handles.Plot_Button,'string','STOP')
  %datafile = fopen('data.csv','wt');
  % initialize data matrices
  position = int16.empty;
  velocity = int16.empty;
  angle = uint8.empty;
  actual_angle = single.empty;
  time = single.empty;
  i = 1;
  % start timing
  tstart = tic; 
  
  while RUN == 1
    % clear the input buffer
    flushinput(comport);
    % wait for sync character to be received from MCU
    if fread(comport,1,'int8') == 'A'
        % record time when data received since program start
        time = [time, toc(tstart)];
        % fetch data as single 8-bit bytes 
        pos1 = int16(fread(comport,1,'int8'));
        pos2 = int16(fread(comport,1,'int8'));  
        angle = [angle, fread(comport,1,'uint8')];
        % join the two position bytes to the actual 16-bit value
        for b = 9:16
            pos2 = bitset(pos2,b,0);
        end
        pos = bitor(bitshift(pos1,8),pos2,'int16'); 
        position = [position, pos/100];
        % remove outliers
        if position(i) > 20
            position(i) = position(i-1);
            angle(i) = angle(i-1);
        end
        % convert angle sensor output to actual angle
        actual_angle = [actual_angle, -0.59*single(angle(i))+31];

        % plot position data
        axes(handles.position_plot);
        plot(time, position);
        hline = yline(targetposition); 
        xlabel('Time (s)')
        ylabel('Position (cm)')
        axis ([0 inf -1 20])

        % plot angle data
        axes(handles.angle_plot); 
        plot(time, actual_angle);
        xlabel('Time (s)')
        ylabel('Angle (deg)')
        axis ([0 inf -20 20])

        % use drawnow to update the figure
        drawnow  
        % save data
        %%fprintf(datafile,'%f, %f, %i\n',time(i), angle_actual(i), position(i)); 
        i = i + 1;
    end  
  end
else
  RUN = 0;
  % change the string on the button to PLOT
  set(handles.Plot_Button,'string','PLOT')
  %%fclose(datafile);
end
%---------------------------------------------------------------------- 


% --- Executes on button press in Update_Variables---------------------
function Update_Variables_Callback(hObject, eventdata, handles)
global comport

flushinput(comport);
fprintf(comport,'%c','A'); %sync character
fprintf(comport,'%c',str2num(get(handles.KPX_Value,'string')));
fprintf(comport,'%c',str2num(get(handles.KIX_Value,'string')));
fprintf(comport,'%c',str2num(get(handles.KDX_Value,'string')));
fprintf(comport,'%c',str2num(get(handles.KPA_Value,'string')));
fprintf(comport,'%c',str2num(get(handles.KDA_Value,'string')));
fprintf(comport,'%c',get(handles.setpoint_slider,'Value'));

set(handles.KPX_return, 'String', get(handles.KPX_Value,'string'))
set(handles.KIX_return, 'String', get(handles.KIX_Value,'string'));
set(handles.KDX_return, 'String', get(handles.KDX_Value,'string'));
set(handles.KPA_return, 'String', get(handles.KPA_Value,'string'));
set(handles.KDA_return, 'String', get(handles.KDA_Value,'string'));
%----------------------------------------------------------------------

% --- Executes on slider movement.
function setpoint_slider_Callback(hObject, eventdata, handles)
global hline
global targetposition
%get rounded value from slider
targetposition = round(get(handles.setpoint_slider,'Value'));
%set slider value to current rounded value
set(handles.setpoint_slider,'Value', targetposition);
%update target position readout
set(handles.setpoint_text,'string',num2str(targetposition));
% disp(num2str(get(handles.setpoint_slider,'Value')));
axes(handles.position_plot);
delete(hline);
% plot horizontal line showing target position on position plot
hline = yline(targetposition); 



% --- Executes during object creation, after setting all properties.
function setpoint_slider_CreateFcn(hObject, eventdata, handles)
% Hint: slider controls usually have a light gray background.
global hline ;
% axes(handles.position_plot);
hline = yline(-1000);
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
%----------------------------------------------------------------------


function KPX_Value_Callback(hObject, eventdata, handles)
function KPX_Value_CreateFcn(hObject, eventdata, handles)

function KIX_Value_Callback(hObject, eventdata, handles)
function KIX_Value_CreateFcn(hObject, eventdata, handles)

function KDX_Value_Callback(hObject, eventdata, handles)
function KDX_Value_CreateFcn(hObject, eventdata, handles)

function setpoint_text_CreateFcn(hObject, eventdata, handles)

function KPA_Value_Callback(hObject, eventdata, handles)
function KPA_Value_CreateFcn(hObject, eventdata, handles)

function KDA_Value_Callback(hObject, eventdata, handles)
function KDA_Value_CreateFcn(hObject, eventdata, handles)

