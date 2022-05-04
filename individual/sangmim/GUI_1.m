function varargout = GUI_1(varargin)
% GUI_1 MATLAB code for GUI_1.fig
%      GUI_1, by itself, creates a new GUI_1 or raises the existing
%      singleton*.
%
%      H = GUI_1 returns the handle to a new GUI_1 or the handle to
%      the existing singleton*.
%
%      GUI_1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_1.M with the given input arguments.
%
%      GUI_1('Property','Value',...) creates a new GUI_1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_1

% Last Modified by GUIDE v2.5 04-May-2022 08:25:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_1_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_1_OutputFcn, ...
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


% --- Executes just before GUI_1 is made visible.
function GUI_1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_1 (see VARARGIN)

% Choose default command line output for GUI_1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI_1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function txt_theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to txt_theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_theta_1 as text
%        str2double(get(hObject,'String')) returns contents of txt_theta_1 as a double


% --- Executes during object creation, after setting all properties.
function txt_theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to txt_theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_theta_2 as text
%        str2double(get(hObject,'String')) returns contents of txt_theta_2 as a double


% --- Executes during object creation, after setting all properties.
function txt_theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to txt_theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_theta_3 as text
%        str2double(get(hObject,'String')) returns contents of txt_theta_3 as a double


% --- Executes during object creation, after setting all properties.
function txt_theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_theta_4_Callback(hObject, eventdata, handles)
% hObject    handle to txt_theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_theta_4 as text
%        str2double(get(hObject,'String')) returns contents of txt_theta_4 as a double


% --- Executes during object creation, after setting all properties.
function txt_theta_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_theta_5_Callback(hObject, eventdata, handles)
% hObject    handle to txt_theta_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_theta_5 as text
%        str2double(get(hObject,'String')) returns contents of txt_theta_5 as a double


% --- Executes during object creation, after setting all properties.
function txt_theta_5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_theta_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_theta_6_Callback(hObject, eventdata, handles)
% hObject    handle to txt_theta_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_theta_6 as text
%        str2double(get(hObject,'String')) returns contents of txt_theta_6 as a double


% --- Executes during object creation, after setting all properties.
function txt_theta_6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_theta_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_pos_x_Callback(hObject, eventdata, handles)
% hObject    handle to txt_pos_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_pos_x as text
%        str2double(get(hObject,'String')) returns contents of txt_pos_x as a double


% --- Executes during object creation, after setting all properties.
function txt_pos_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_pos_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_pos_y_Callback(hObject, eventdata, handles)
% hObject    handle to txt_pos_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_pos_y as text
%        str2double(get(hObject,'String')) returns contents of txt_pos_y as a double


% --- Executes during object creation, after setting all properties.
function txt_pos_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_pos_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_pos_z_Callback(hObject, eventdata, handles)
% hObject    handle to txt_pos_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_pos_z as text
%        str2double(get(hObject,'String')) returns contents of txt_pos_z as a double


% --- Executes during object creation, after setting all properties.
function txt_pos_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_pos_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_forward.
function btn_forward_Callback(hObject, eventdata, handles)
% hObject    handle to btn_forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Th_1 = str2double(handles.txt_theta_1.String)*pi/180;
Th_2 = str2double(handles.txt_theta_2.String)*pi/180;
Th_3 = str2double(handles.txt_theta_3.String)*pi/180;
Th_4 = str2double(handles.txt_theta_4.String)*pi/180;
Th_5 = str2double(handles.txt_theta_5.String)*pi/180;
Th_6 = str2double(handles.txt_theta_6.String)*pi/180;

L_1 = 20;
L_2 = 50;
L_3 = 40;
L_4 = 40;
L_5 = 40;
L_6 = 40;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);
L(5) = Link([0 0 L_5 0]);
L(6) = Link([0 0 L_6 0]);

Robot = SerialLink(L);
Robot.name = 'RRR_Robot';
Robot.plot([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]);

T = Robot.fkine([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]);

handles.txt_pos_x.String = num2str(floor(T(1,4)));
handles.txt_pos_y.String = num2str(floor(T(2,4)));
handles.txt_pos_z.String = num2str(floor(T(3,4)));


% --- Executes on button press in btn_inverse.
function btn_inverse_Callback(hObject, eventdata, handles)
% hObject    handle to btn_inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

PX = str2double(handles.txt_pos_x.String);
PY = str2double(handles.txt_pos_y.String);
PZ = str2double(handles.txt_pos_z.String);

L_1 = 20;
L_2 = 50;
L_3 = 40;
L_4 = 40;
L_5 = 40;
L_6 = 40;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);
L(5) = Link([0 0 L_5 0]);
L(6) = Link([0 0 L_6 0]);


Robot = SerialLink(L);
Robot.name = 'RRR_Robot';

T = [ 1 0 0 PX;
      0 1 0 PY;
      0 0 1 PZ;
      0 0 0 1];
  
J = Robot.ikine(T, [0 0 0], [1 1 1 0 0 0]) * 180/pi;
handles.txt_theta_1.String = num2str(floor(J(1)));
handles.txt_theta_2.String = num2str(floor(J(2)));
handles.txt_theta_3.String = num2str(floor(J(3)));
handles.txt_theta_4.String = num2str(floor(J(4)));
handles.txt_theta_5.String = num2str(floor(J(5)));
handles.txt_theta_6.String = num2str(floor(J(6)));

Robot.plot(J*pi/180);
