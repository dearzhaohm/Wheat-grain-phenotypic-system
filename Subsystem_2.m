 function varargout = Subsystem_2(varargin)
% Subsystem_2 MATLAB code for Subsystem_2.fig
%      Subsystem_2, by itself, creates a new Subsystem_2 or raises the existing
%      singleton*.
%
%      H = Subsystem_2 returns the handle to a new Subsystem_2 or the handle to
%      the existing singleton*.
%
%      Subsystem_2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in Subsystem_2.M with the given input arguments.
%
%      Subsystem_2('Property','Value',...) creates a new Subsystem_2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Subsystem_2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Subsystem_2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Subsystem_2

% Last Modified by GUIDE v2.5 16-Jul-2020 21:15:06

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Subsystem_2_OpeningFcn, ...
                   'gui_OutputFcn',  @Subsystem_2_OutputFcn, ...
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


% --- Executes just before Subsystem_2 is made visible.
function Subsystem_2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Subsystem_2 (see VARARGIN)

% Choose default command line output for Subsystem_2
handles.output = hObject;
global clickcounts
clickcounts = 0;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Subsystem_2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Subsystem_2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现
z=importdata('mat1.mat');                      %全部种子阈值分割后的二值图像
z1 = importdata('mat2.mat');                   %全部种子原始图像
% grayimg = rgb2gray(z);                            %将原图像灰度化
f2=im2double(z);                             %将二值图像转换为双精度型图像 
h=graythresh(f2);                                  %自动阈值分割
K=im2bw(f2,h); 
K=imfill(K,'holes');
f3=K;
%先闭运算 再开运算
se=strel('disk',5);
L = imclose(f3,se);                                                  %对小麦种子进行开运算
f4 = imopen(L,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(f4,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
%M是大小和BWing一样的图像数组，里面存放着对bwing图像的标签值（即判定为连通后，在L矩阵中标记出来） 
% figure('NumberTitle','off','Name','Regional marking of wheat seeds'),      %小麦种子的区域标记
% subplot(121),imshow(f4);title('Morphologically processed image')           %显示形态学操作后的图像
% subplot(122),imshow(L);title('Tagged image')                               %显示标记后的图像  
global status N1
status=regionprops(L,'BoundingBox');
% centroid=regionprops(L,'Centroid');
% %返回值STATS是一个长度为max(L(:))的结构数组，结构数组的相应域定义了每一个区域相应属性下的度量
% for i=1:num
%     rectangle('position',status(i).BoundingBox,'edgecolor','r');
%     %参数说明：position绘制的为二维图像（他是通过对角的两点确定矩形框）
%     %edgecolor 指边缘图像，r表示变换为红色。
%     %facecolor 指内部填充颜色。
%     text(centroid(i,1).Centroid(1,1)-10,centroid(i,1).Centroid(1,2)-20, num2str(i),'Color', 'r')
%     %这个是为绘制出来的矩形框图标记数字
% end
copy_L=L;
L_part1=(copy_L==1);                                            %进行区域的选择，此处只保留1
L_part2=(copy_L==N1);                                            %进行区域的选择，此处只保留2
% L_part2=(copy_L==3);                                            %进行区域的选择，此处只保留3
% image_part1 = (mark_image ~= 1);
axes(handles.axes1);
imshow(L_part2);title('Separated seed binary image ');               %显示分离后的二值图像
handles.f3=L_part1;                                                                  %分离的硬币二值图像
handles.f=L_part2;                                                                   %分离第一个小麦的二值图像
handles.er=z;                                                                        %全部种子阈值分割后的二值图像                                                  
handles.f1=z1;                                                                       %全部种子原始图像
guidata(hObject,handles);



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global N1    %定义全局变量N
input = str2num(get(handles.edit1,'String')); 
if (isempty(input))
 set(handles.edit1,'String','1')   %检出输入是否为空，如果为空或是输入非数字字符，则默认显示为1
end
N1 = input;
% disp(N);
guidata(hObject, handles); 


% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现    
f1=handles.f;                                    %分离第一个小麦的二值图像
f2 = handles.f3;                                 %分离的硬币二值图像
total=bwarea(f1);                                 %小麦的像素面积
total2=bwarea(f2);                               %硬币的像素面积
c = 551.266;                                     %硬币的实际面积
p = (c*total)/total2;                            %小麦的实际面积                       
totalp=num2str(p);
set(handles.text2,'string',totalp);
handles.total1=totalp;
guidata(hObject,handles);

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
f=handles.f;                                     %分离第一个小麦的二值图像
f1 = handles.f3;                                 %分离的硬币二值图像
L=logical(f);
L1 = logical(f1);
format long;
a=regionprops(L,'MajorAxisLength');             %计算小麦像素的长颈
a1 = regionprops(L1,'MajorAxisLength');         %计算硬币像素的直径
a=struct2cell(a);
a1 = struct2cell(a1);
a=cell2mat(a);
a1 = cell2mat(a1);
c = 25;                                        %硬币的实际直径
p = (c*a)/a1;                                  %小麦的实际长颈
% f=texth=cell2mat(h)
% wrap(g,h);
% a1=a(1),a2=a(2),a3=a(3),a4=a(4)
% b1=b(1),b2=b(2),b3=b(3),b4=b(4)
set(handles.text3,'string',p);
handles.L1 = p;
guidata(hObject,handles);

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
f=handles.f;                                     %分离第一个小麦的二值图像
f1 = handles.f3;                                 %分离的硬币二值图像
S=logical(f);
S1=logical(f1);
format long;
b=regionprops(S,'MinorAxisLength');              %计算小麦像素短颈
b1 = regionprops(S1,'MinorAxisLength');          %计算硬币像素直径
b=struct2cell(b);
b1 = struct2cell(b1);
% g = uicontrol('style', 'text', 'position', [20 20 100 100]);
b=cell2mat(b);
b1 = cell2mat(b1);
c = 25;
p = (c*b)/b1;
% f=texth=cell2mat(h)
% wrap(g,h);
% a1=a(1),a2=a(2),a3=a(3),a4=a(4)
% b1=b(1),b2=b(2),b3=b(3),b4=b(4)
set(handles.text4,'string',p);
handles.S1 = p;
guidata(hObject,handles);



% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
f=handles.f;                                                         %分离第一个小麦的二值图像                                                     
[J, thresh]=edge(f, 'sobel', [], 'horizontal');                      %使用sobel算子进行边缘提取
[K, thresh]=edge(f, 'canny');                                       %使用canny算子进行边缘提取
[S, thresh]=edge(f, 'log', [], 2.3);                                %使用log算子进行边缘提取 
% H=bwperim(f,8);                                                   %采用函数bwperim获取二值图像的边缘
H=bwmorph(f,'remove');                                              %移除二值图像内部的像素点 
se=strel('disk', 2);                                                 %先膨胀后腐蚀进行边缘提取
M=imdilate(f, se);
N=imerode(f, se);
L=M-N;
figure('NumberTitle','off','Name','Edge extraction of the first wheat seed'),        %第一粒小麦种子的边缘提取
subplot(221);  imshow(J);title('Image using sobel operator for edge extraction')      %显示使用sobel算子进行边缘提取的图像
subplot(222);  imshow(K);title('Image using canny operator for edge extraction')      %显示使用canny算子进行边缘提取的图像
subplot(223);  imshow(S);title('Image using log operator for edge extraction')        %显示使用log算子进行边缘提取的图像
subplot(224);  imshow(L);title('Dilate and erode the image for edge extraction')      %显示先膨胀后腐蚀进行边缘提取的图像
handles.L1=L;
guidata(hObject,handles);


function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global N2    %定义全局变量N
input = str2num(get(handles.edit2,'String')); 
if (isempty(input))
 set(handles.edit2,'String','1')   %检出输入是否为空，如果为空或是输入非数字字符，则默认显示为1
end
N2 = input;
% disp(N);
guidata(hObject, handles); 

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global N3    %定义全局变量N
input = str2num(get(handles.edit3,'String')); 
if (isempty(input))
 set(handles.edit3,'String','1')   %检出输入是否为空，如果为空或是输入非数字字符，则默认显示为1
end
N3 = input;
% disp(N);
guidata(hObject, handles); 

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现
z=importdata('mat1.mat');                      %全部种子阈值分割后的二值图像                                                  
% grayimg = rgb2gray(z);                       %将原图像灰度化
f2=im2double(z);                               %将二值图像转换为双精度型图像 
h=graythresh(f2);                              %自动阈值分割
K=im2bw(f2,h); 
K=imfill(K,'holes');
f3=K;
%先闭运算 再开运算
se=strel('disk',5);
L = imclose(f3,se);                                                  %对小麦种子进行开运算
f4 = imopen(L,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(f4,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
%M是大小和BWing一样的图像数组，里面存放着对bwing图像的标签值（即判定为连通后，在L矩阵中标记出来） 
% figure('NumberTitle','off','Name','Regional marking of wheat seeds'),      %小麦种子的区域标记
% subplot(121),imshow(f4);title('Morphologically processed image')           %显示形态学操作后的图像
% subplot(122),imshow(L);title('Tagged image')                               %显示标记后的图像  
global status N2
status=regionprops(L,'BoundingBox');
% centroid=regionprops(L,'Centroid');
% %返回值STATS是一个长度为max(L(:))的结构数组，结构数组的相应域定义了每一个区域相应属性下的度量
% for i=1:num
%     rectangle('position',status(i).BoundingBox,'edgecolor','r');
% %参数说明：position绘制的为二维图像（他是通过对角的两点确定矩形框）
% %edgecolor 指边缘图像，r表示变换为红色。
% %facecolor 指内部填充颜色。
%     text(centroid(i,1).Centroid(1,1)-10,centroid(i,1).Centroid(1,2)-20, num2str(i),'Color', 'r')
% %这个是为绘制出来的矩形框图标记数字
% end
copy_L=L;
L_part2=(copy_L==N2);                                            %进行区域的选择，此处只保留2
% image_part1 = (mark_image ~= 1);
axes(handles.axes2);
imshow(L_part2);title('Separated seed binary image');              %显示分离后的二值图像
handles.d2=L_part2;                                                                 %分离第二个小麦的二值图像
guidata(hObject,handles);



% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现    
f1=handles.d2;                                   %分离第二个小麦的二值图像
f2 = handles.f3;                                 %分离的硬币二值图像
total=bwarea(f1);                                 %小麦的像素面积
total2=bwarea(f2);                               %硬币的像素面积
c = 551.266;                                     %硬币的实际面积
p = (c*total)/total2;                            %小麦的实际面积                       
totalp=num2str(p);
set(handles.text5,'string',totalp);
handles.total2=totalp;
guidata(hObject,handles);


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
f=handles.d2;                                     %分离第二个小麦的二值图像
f1 = handles.f3;                                 %分离的硬币二值图像
L=logical(f);                                    
L1 = logical(f1);                                
format long;
a=regionprops(L,'MajorAxisLength');             %计算小麦像素的长颈
a1 = regionprops(L1,'MajorAxisLength');         %计算硬币像素的直径
a=struct2cell(a);
a1 = struct2cell(a1);
a=cell2mat(a);
a1 = cell2mat(a1);
c = 25;                                        %硬币的实际直径
p = (c*a)/a1;                                  %小麦的实际长颈
% f=texth=cell2mat(h)
% wrap(g,h);
% a1=a(1),a2=a(2),a3=a(3),a4=a(4)
% b1=b(1),b2=b(2),b3=b(3),b4=b(4)
set(handles.text6,'string',p);
handles.L2 = p;
guidata(hObject,handles);


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
f=handles.d2;                                     %分离第一个小麦的二值图像
f1 = handles.f3;                                 %分离的硬币二值图像
S=logical(f);
S1=logical(f1);
format long;
b=regionprops(S,'MinorAxisLength');              %计算小麦像素短颈
b1 = regionprops(S1,'MinorAxisLength');          %计算硬币像素直径
b=struct2cell(b);
b1 = struct2cell(b1);
% g = uicontrol('style', 'text', 'position', [20 20 100 100]);
b=cell2mat(b);
b1 = cell2mat(b1);
c = 25;
p = (c*b)/b1;
% f=texth=cell2mat(h)
% wrap(g,h);
% a1=a(1),a2=a(2),a3=a(3),a4=a(4)
% b1=b(1),b2=b(2),b3=b(3),b4=b(4)
set(handles.text7,'string',p);
handles.S2 = p;
guidata(hObject,handles);





% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
f=handles.d2;
[J, thresh]=edge(f, 'sobel', [], 'horizontal');                      %使用sobel算子进行边缘提取
[K, thresh]=edge(f, 'canny');                                       %使用canny算子进行边缘提取
[S, thresh]=edge(f, 'log', [], 2.3);                                %使用log算子进行边缘提取 
% H=bwperim(f,8);                                                   %采用函数bwperim获取二值图像的边缘
H=bwmorph(f,'remove');                                              %移除二值图像内部的像素点 
se=strel('disk', 2);                                                 %先膨胀后腐蚀进行边缘提取
M=imdilate(f, se);
N=imerode(f, se);
L=M-N;
figure('NumberTitle','off','Name','Edge extraction of the second wheat seed'),        %第二粒小麦种子的边缘提取
subplot(221);  imshow(J);title('Image using sobel operator for edge extraction')      %显示使用sobel算子进行边缘提取的图像
subplot(222);  imshow(K);title('Image using canny operator for edge extraction')      %显示使用canny算子进行边缘提取的图像
subplot(223);  imshow(S);title('Image using log operator for edge extraction')        %显示使用log算子进行边缘提取的图像
subplot(224);  imshow(L);title('Dilate and erode the image for edge extraction')      %显示先膨胀后腐蚀进行边缘提取的图像
handles.L2=L;
guidata(hObject,handles);

% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现     
z=importdata('mat1.mat');                      %全部种子阈值分割后的二值图像                                                  
f2=im2double(z);                               %将二值图像转换为双精度型图像 
h=graythresh(f2);                              %自动阈值分割
K=im2bw(f2,h); 
K=imfill(K,'holes');
f3=K;
%先闭运算 再开运算
se=strel('disk',5);
L = imclose(f3,se);                                                  %对小麦种子进行开运算
f4 = imopen(L,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(f4,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
%M是大小和BWing一样的图像数组，里面存放着对bwing图像的标签值（即判定为连通后，在L矩阵中标记出来） 
% figure('NumberTitle','off','Name','Regional marking of wheat seeds'),      %小麦种子的区域标记
% subplot(121),imshow(f4);title('Morphologically processed image')           %显示形态学操作后的图像
% subplot(122),imshow(L);title('Tagged image')                               %显示标记后的图像   
global status N3
status=regionprops(L,'BoundingBox');
% centroid=regionprops(L,'Centroid');
% %返回值STATS是一个长度为max(L(:))的结构数组，结构数组的相应域定义了每一个区域相应属性下的度量
% for i=1:num
%     rectangle('position',status(i).BoundingBox,'edgecolor','r');
%     %参数说明：position绘制的为二维图像（他是通过对角的两点确定矩形框）
%     %edgecolor 指边缘图像，r表示变换为红色。
%     %facecolor 指内部填充颜色。
%     text(centroid(i,1).Centroid(1,1)-10,centroid(i,1).Centroid(1,2)-20, num2str(i),'Color', 'r') %这个是为绘制出来的矩形框图标记数字
% end
copy_L=L;
L_part3=(copy_L==N3);                                            %进行区域的选择，此处只保留2
% image_part1 = (mark_image ~= 1);
axes(handles.axes3);
imshow(L_part3);title('Separated seed binary image');              %显示分离后的二值图像
handles.d3=L_part3;                                                                 %分离第三个小麦的二值图像
guidata(hObject,handles);



% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现    
f1=handles.d3;                                   %分离第三个小麦的二值图像
f2 = handles.f3;                                 %分离的硬币二值图像
total=bwarea(f1);                                 %小麦的像素面积
total2=bwarea(f2);                               %硬币的像素面积
c = 551.266;                                     %硬币的实际面积
p = (c*total)/total2;                            %小麦的实际面积                       
totalp=num2str(p);
set(handles.text8,'string',totalp);
handles.total3=totalp;
guidata(hObject,handles);



% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
f=handles.d3;                                    %分离第一个小麦的二值图像
f1 = handles.f3;                                 %分离的硬币二值图像
L=logical(f);
L1 = logical(f1);
format long;
a=regionprops(L,'MajorAxisLength');             %计算小麦像素的长颈
a1 = regionprops(L1,'MajorAxisLength');         %计算硬币像素的直径
a=struct2cell(a);
a1 = struct2cell(a1);
a=cell2mat(a);
a1 = cell2mat(a1);
c = 25;                                        %硬币的实际直径
p = (c*a)/a1;                                  %小麦的实际长颈
% f=texth=cell2mat(h)
% wrap(g,h);
% a1=a(1),a2=a(2),a3=a(3),a4=a(4)
% b1=b(1),b2=b(2),b3=b(3),b4=b(4)
set(handles.text9,'string',p);
handles.L3 = p;
guidata(hObject,handles);




% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
f=handles.d3;                                     %分离第三个小麦的二值图像
f1 = handles.f3;                                 %分离的硬币二值图像
S=logical(f);
S1=logical(f1);
format long;
b=regionprops(S,'MinorAxisLength');              %计算小麦像素短颈
b1 = regionprops(S1,'MinorAxisLength');          %计算硬币像素直径
b=struct2cell(b);
b1 = struct2cell(b1);
% g = uicontrol('style', 'text', 'position', [20 20 100 100]);
b=cell2mat(b);
b1 = cell2mat(b1);
c = 25;
p = (c*b)/b1;
% f=texth=cell2mat(h)
% wrap(g,h);
% a1=a(1),a2=a(2),a3=a(3),a4=a(4)
% b1=b(1),b2=b(2),b3=b(3),b4=b(4)
set(handles.text10,'string',p);
handles.S3 = p;
guidata(hObject,handles);






% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
f=handles.d3;
[J, thresh]=edge(f, 'sobel', [], 'horizontal');                      %使用sobel算子进行边缘提取
[K, thresh]=edge(f, 'canny');                                       %使用canny算子进行边缘提取
[S, thresh]=edge(f, 'log', [], 2.3);                                %使用log算子进行边缘提取 
% H=bwperim(f,8);                                                   %采用函数bwperim获取二值图像的边缘
H=bwmorph(f,'remove');                                              %移除二值图像内部的像素点 
se=strel('disk', 2);                                                 %先膨胀后腐蚀进行边缘提取
M=imdilate(f, se);
N=imerode(f, se);
L=M-N;
figure('NumberTitle','off','Name','Edge extraction of the third wheat seed'),        %第三粒小麦种子的边缘提取
subplot(221);  imshow(J);title('Image using sobel operator for edge extraction')      %显示使用sobel算子进行边缘提取的图像
subplot(222);  imshow(K);title('Image using canny operator for edge extraction')      %显示使用canny算子进行边缘提取的图像
subplot(223);  imshow(S);title('Image using log operator for edge extraction')        %显示使用log算子进行边缘提取的图像
subplot(224);  imshow(L);title('Dilate and erode the image for edge extraction')      %显示先膨胀后腐蚀进行边缘提取的图像
handles.L3=L;
guidata(hObject,handles);


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                                       %取消警告
feature jit off                                                   %加速代码实现
f = handles.f2;                                                   %种子分离后的RGB图像

rgb = f;                             %读取种子分离后的RGB图片
r = rgb(:,:,1);
g = rgb(:,:,2);
b = rgb(:,:,3);
R= medfilt2(r);
G = medfilt2(g);
B = medfilt2(b);
I= cat(3,R,G,B);                    %中值滤波处理
I=rgb2gray(I);
disp(I);
I=imresize(I,1/40);
[M,N]=size(I);
for i=1:M
    for j=1:N
        for n=1:16
            if(n-1)*16<=I(i,j)&I(i,j)<=(n-1)*16+15
                I(i,j)=n-1;
            end
        end
    end
end
P=zeros(16,16,4);
for m=1:16
    for n=1:16
        for i=1:M
            for j=1:N
                if j<N & I(i,j)==m-1 & I(i,j+1)==n-1
                    P(m,n,1)=P(m,n,1)+1;
                end
                if i>1&j<N&I(i,j)==m-1 & I(i-1,j+1)==n-1
                    P(m,n,2)=P(m,n,2)+1;
                end
                if i<M & I(i,j)==m-1 & I(i+1,j)==n-1
                    P(m,n,3)=P(m,n,3)+1;
                end
                if i<M &j<N & I(i,j)==m-1 & I(i+1,j+1)==n-1
                    P(m,n,4)=P(m,n,4)+1;
                end
            end
        end
    end
end
for n=1:4
    P(:,:,n)=P(:,:,n)/sum(sum(P(:,:,n)));
end
H=zeros(1,4);
CON=H;
Ux=H;Uy=H;
deltaX=H; deltaY=H;
COR=H;
L=H;
for n=1:4
    ASM(n)=sum(sum(P(:,:,n).^2));
    for i=1:16
        for j=1:16
            if P(i,j,n)~=0
                H(n)=-P(i,j,n)*log(P(i,j,n))+H(n);
            end
            CON(n)=(i-j)^2*P(i,j,n)+CON(n);
            Ux(n)=i*P(i,j,n)+Ux(n);
            Uy(n)=j*P(i,j,n)+Ux(n);
        end
    end
end
for n=1:4
    for i=1:16
        for j=1:16
            deltaX(n)=(i-Ux(n))^2*P(i,j,n)+deltaX(n);
            deltaY(n)=(j-Uy(n))^2*P(i,j,n)+deltaY(n);
            COR(n)=i*j*P(i,j,n)+COR(n);
            L(n)=P(i,j,n)^2/(1+(i-j)^2)+L(n);
        end
    end
    COR(n)=(COR(n)-Ux(n)*Uy(n))/deltaX(n)/deltaY(n);
end
T=[ASM(1),ASM(2),ASM(3),ASM(4)];   %能量
t(1,:)=T;
U=[H(1),H(2),H(3),H(4)];           %熵
u(1,:)=U;
V=[CON(1),CON(2),CON(3),CON(4)];   %惯性矩
v(1,:)=V;
W=[COR(1),COR(2),COR(3),COR(4)];   %相关性
w(1,:)=W;
Z=[L(1),L(2),L(3),L(4)];           %逆差距
z(1,:)=Z; 



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global N4    %定义全局变量N
input = str2num(get(handles.edit4,'String')); 
if (isempty(input))
 set(handles.edit4,'String','1')   %检出输入是否为空，如果为空或是输入非数字字符，则默认显示为1
end
N4 = input;
% disp(N);
guidata(hObject, handles); 
% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global N5    %定义全局变量N
input = str2num(get(handles.edit5,'String')); 
if (isempty(input))
 set(handles.edit5,'String','1')   %检出输入是否为空，如果为空或是输入非数字字符，则默认显示为1
end
N5 = input;
% disp(N);
guidata(hObject, handles); 
% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global N6    %定义全局变量N
input = str2num(get(handles.edit6,'String')); 
if (isempty(input))
 set(handles.edit6,'String','1')   %检出输入是否为空，如果为空或是输入非数字字符，则默认显示为1
end
N6 = input;
% disp(N);
guidata(hObject, handles); 
% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现     
z=importdata('mat1.mat');                      %全部种子阈值分割后的二值图像                                                
% grayimg = rgb2gray(z);                       %将原图像灰度化
f2=im2double(z);                               %将二值图像转换为双精度型图像 
h=graythresh(f2);                              %自动阈值分割
K=im2bw(f2,h); 
K=imfill(K,'holes');
f3=K;
%先闭运算 再开运算
se=strel('disk',5);
L = imclose(f3,se);                                                  %对小麦种子进行开运算
f4 = imopen(L,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(f4,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
%M是大小和BWing一样的图像数组，里面存放着对bwing图像的标签值（即判定为连通后，在L矩阵中标记出来） 
% figure('NumberTitle','off','Name','Regional marking of wheat seeds'),      %小麦种子的区域标记
% subplot(121),imshow(f4);title('Morphologically processed image')           %显示形态学操作后的图像
% subplot(122),imshow(L);title('Tagged image')                               %显示标记后的图像  
global status N4
status=regionprops(L,'BoundingBox');
% centroid=regionprops(L,'Centroid');
% %返回值STATS是一个长度为max(L(:))的结构数组，结构数组的相应域定义了每一个区域相应属性下的度量
% for i=1:num
%     rectangle('position',status(i).BoundingBox,'edgecolor','r');
%     %参数说明：position绘制的为二维图像（他是通过对角的两点确定矩形框）
%     %edgecolor 指边缘图像，r表示变换为红色。
%     %facecolor 指内部填充颜色。
%     text(centroid(i,1).Centroid(1,1)-10,centroid(i,1).Centroid(1,2)-20, num2str(i),'Color', 'r')%这个是为绘制出来的矩形框图标记数字
% end
copy_L=L;
L_part4=(copy_L==N4);                                            %进行区域的选择，此处只保留2
% image_part1 = (mark_image ~= 1);
axes(handles.axes4);
imshow(L_part4);title('Separated seed binary image');              %显示分离后的二值图像
handles.d4=L_part4;                                             %分离第四个小麦的二值图像
guidata(hObject,handles);




% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现    
f1=handles.d4;                                   %分离第四个小麦的二值图像
f2 = handles.f3;                                 %分离的硬币二值图像
total=bwarea(f1);                                 %小麦的像素面积
total2=bwarea(f2);                               %硬币的像素面积
c = 551.266;                                     %硬币的实际面积
p = (c*total)/total2;                            %小麦的实际面积                       
totalp=num2str(p);
set(handles.text11,'string',totalp);
handles.total4=totalp;
guidata(hObject,handles);




% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
f=handles.d4;                                     %分离第四个小麦的二值图像
f1 = handles.f3;                                 %分离的硬币二值图像
L=logical(f);                                    
L1 = logical(f1);                                
format long;
a=regionprops(L,'MajorAxisLength');             %计算小麦像素的长颈
a1 = regionprops(L1,'MajorAxisLength');         %计算硬币像素的直径
a=struct2cell(a);
a1 = struct2cell(a1);
a=cell2mat(a);
a1 = cell2mat(a1);
c = 25;                                        %硬币的实际直径
p = (c*a)/a1;                                  %小麦的实际长颈
% f=texth=cell2mat(h)
% wrap(g,h);
% a1=a(1),a2=a(2),a3=a(3),a4=a(4)
% b1=b(1),b2=b(2),b3=b(3),b4=b(4)
set(handles.text12,'string',p);
handles.L4 = p;
guidata(hObject,handles);



% --- Executes on button press in pushbutton23.
function pushbutton23_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
f=handles.d4;                                     %分离第四个小麦的二值图像
f1 = handles.f3;                                 %分离的硬币二值图像
S=logical(f);
S1=logical(f1);
format long;
b=regionprops(S,'MinorAxisLength');              %计算小麦像素短颈
b1 = regionprops(S1,'MinorAxisLength');          %计算硬币像素直径
b=struct2cell(b);
b1 = struct2cell(b1);
% g = uicontrol('style', 'text', 'position', [20 20 100 100]);
b=cell2mat(b);
b1 = cell2mat(b1);
c = 25;
p = (c*b)/b1;
% f=texth=cell2mat(h)
% wrap(g,h);
% a1=a(1),a2=a(2),a3=a(3),a4=a(4)
% b1=b(1),b2=b(2),b3=b(3),b4=b(4)
set(handles.text13,'string',p);
handles.S4 = p;
guidata(hObject,handles);








% --- Executes on button press in pushbutton31.
function pushbutton31_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
f=handles.d5;
[J, thresh]=edge(f, 'sobel', [], 'horizontal');                      %使用sobel算子进行边缘提取
[K, thresh]=edge(f, 'canny');                                       %使用canny算子进行边缘提取
[S, thresh]=edge(f, 'log', [], 2.3);                                %使用log算子进行边缘提取 
% H=bwperim(f,8);                                                   %采用函数bwperim获取二值图像的边缘
H=bwmorph(f,'remove');                                              %移除二值图像内部的像素点 
se=strel('disk', 2);                                                 %先膨胀后腐蚀进行边缘提取
M=imdilate(f, se);
N=imerode(f, se);
L=M-N;
figure('NumberTitle','off','Name','Edge extraction of the fifth wheat seed'),        %第五粒小麦种子的边缘提取
subplot(221);  imshow(J);title('Image using sobel operator for edge extraction')      %显示使用sobel算子进行边缘提取的图像
subplot(222);  imshow(K);title('Image using canny operator for edge extraction')      %显示使用canny算子进行边缘提取的图像
subplot(223);  imshow(S);title('Image using log operator for edge extraction')        %显示使用log算子进行边缘提取的图像
subplot(224);  imshow(L);title('Dilate and erode the image for edge extraction')      %显示先膨胀后腐蚀进行边缘提取的图像
handles.L5=L;
guidata(hObject,handles);

% --- Executes on button press in pushbutton32.
function pushbutton32_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现     
z=importdata('mat1.mat');                      %全部种子阈值分割后的二值图像                                                
% grayimg = rgb2gray(z);                       %将原图像灰度化
f2=im2double(z);                               %将灰度图像转换为双精度型图像 
h=graythresh(f2);                              %自动阈值分割
K=im2bw(f2,h); 
K=imfill(K,'holes');
f3=K;
%先闭运算 再开运算
se=strel('disk',5);
L = imclose(f3,se);                                                  %对小麦种子进行开运算
f4 = imopen(L,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(f4,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
%M是大小和BWing一样的图像数组，里面存放着对bwing图像的标签值（即判定为连通后，在L矩阵中标记出来） 
% figure('NumberTitle','off','Name','Regional marking of wheat seeds'),      %小麦种子的区域标记
% subplot(121),imshow(f4);title('Morphologically processed image')           %显示形态学操作后的图像
% subplot(122),imshow(L);title('Tagged image')                               %显示标记后的图像  

global status N6
status=regionprops(L,'BoundingBox');
% centroid=regionprops(L,'Centroid');
% %返回值STATS是一个长度为max(L(:))的结构数组，结构数组的相应域定义了每一个区域相应属性下的度量
% for i=1:num
%     rectangle('position',status(i).BoundingBox,'edgecolor','r');
% %参数说明：position绘制的为二维图像（他是通过对角的两点确定矩形框）
% %edgecolor 指边缘图像，r表示变换为红色。
% %facecolor 指内部填充颜色。
%     text(centroid(i,1).Centroid(1,1)-10,centroid(i,1).Centroid(1,2)-20, num2str(i),'Color', 'r')
% %这个是为绘制出来的矩形框图标记数字
% end
copy_L=L;
L_part6=(copy_L==N6);                                            %进行区域的选择，此处只保留2
% image_part1 = (mark_image ~= 1);
axes(handles.axes6);
imshow(L_part6);title('Separated seed binary image');              %显示分离后的二值图像
handles.d6=L_part6;                                             %分离第六个小麦的二值图像
guidata(hObject,handles);



% --- Executes on button press in pushbutton33.
function pushbutton33_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现    
f1=handles.d6;                                   %分离第六个小麦的二值图像
f2 = handles.f3;                                 %分离的硬币二值图像
total=bwarea(f1);                                 %小麦的像素面积
total2=bwarea(f2);                               %硬币的像素面积
c = 551.266;                                     %硬币的实际面积
p = (c*total)/total2;                            %小麦的实际面积                       
totalp=num2str(p);
set(handles.text17,'string',totalp);
handles.total6=totalp;
guidata(hObject,handles);


% --- Executes on button press in pushbutton34.
function pushbutton34_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
f=handles.d6;                                     %分离第六个小麦的二值图像
f1 = handles.f3;                                 %分离的硬币二值图像
L=logical(f);                                    
L1 = logical(f1);                                
format long;
a=regionprops(L,'MajorAxisLength');             %计算小麦像素的长颈
a1 = regionprops(L1,'MajorAxisLength');         %计算硬币像素的直径
a=struct2cell(a);
a1 = struct2cell(a1);
a=cell2mat(a);
a1 = cell2mat(a1);
c = 25;                                        %硬币的实际直径
p = (c*a)/a1;                                  %小麦的实际长颈
% f=texth=cell2mat(h)
% wrap(g,h);
% a1=a(1),a2=a(2),a3=a(3),a4=a(4)
% b1=b(1),b2=b(2),b3=b(3),b4=b(4)
set(handles.text18,'string',p);
handles.L6 = p;
guidata(hObject,handles);



% --- Executes on button press in pushbutton35.
function pushbutton35_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
f=handles.d6;                                     %分离第六个小麦的二值图像
f1 = handles.f3;                                 %分离的硬币二值图像
S=logical(f);
S1=logical(f1);
format long;
b=regionprops(S,'MinorAxisLength');              %计算小麦像素短颈
b1 = regionprops(S1,'MinorAxisLength');          %计算硬币像素直径
b=struct2cell(b);
b1 = struct2cell(b1);
% g = uicontrol('style', 'text', 'position', [20 20 100 100]);
b=cell2mat(b);
b1 = cell2mat(b1);
c = 25;
p = (c*b)/b1;
% f=texth=cell2mat(h)
% wrap(g,h);
% a1=a(1),a2=a(2),a3=a(3),a4=a(4)
% b1=b(1),b2=b(2),b3=b(3),b4=b(4)
set(handles.text19,'string',p);
handles.S6 = p;
guidata(hObject,handles);








% --- Executes on button press in pushbutton37.
function pushbutton37_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton37 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
f=handles.d6;
[J, thresh]=edge(f, 'sobel', [], 'horizontal');                      %使用sobel算子进行边缘提取
[K, thresh]=edge(f, 'canny');                                       %使用canny算子进行边缘提取
[S, thresh]=edge(f, 'log', [], 2.3);                                %使用log算子进行边缘提取 
% H=bwperim(f,8);                                                   %采用函数bwperim获取二值图像的边缘
H=bwmorph(f,'remove');                                              %移除二值图像内部的像素点 
se=strel('disk', 2);                                                 %先膨胀后腐蚀进行边缘提取
M=imdilate(f, se);
N=imerode(f, se);
L=M-N;
figure('NumberTitle','off','Name','Edge extraction of the sixth wheat seed'),        %第六粒小麦种子的边缘提取
subplot(221);  imshow(J);title('Image using sobel operator for edge extraction')      %显示使用sobel算子进行边缘提取的图像
subplot(222);  imshow(K);title('Image using canny operator for edge extraction')      %显示使用canny算子进行边缘提取的图像
subplot(223);  imshow(S);title('Image using log operator for edge extraction')        %显示使用log算子进行边缘提取的图像
subplot(224);  imshow(L);title('Dilate and erode the image for edge extraction')      %显示先膨胀后腐蚀进行边缘提取的图像
handles.L6=L;
guidata(hObject,handles);

% --- Executes on button press in pushbutton38.
function pushbutton38_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton38 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                                       %取消警告
feature jit off                                                   %加速代码实现
f = handles.dd2;                                                   %种子分离后的RGB图像
rgb = f;                             %读取种子分离后的RGB图片
r = rgb(:,:,1);
g = rgb(:,:,2);
b = rgb(:,:,3);
R= medfilt2(r);
G = medfilt2(g);
B = medfilt2(b);
I= cat(3,R,G,B);                    %中值滤波处理
I=rgb2gray(I);
disp(I);
I=imresize(I,1/40);
[M,N]=size(I);
for i=1:M
    for j=1:N
        for n=1:16
            if(n-1)*16<=I(i,j)&I(i,j)<=(n-1)*16+15
                I(i,j)=n-1;
            end
        end
    end
end
P=zeros(16,16,4);
for m=1:16
    for n=1:16
        for i=1:M
            for j=1:N
                if j<N & I(i,j)==m-1 & I(i,j+1)==n-1
                    P(m,n,1)=P(m,n,1)+1;
                end
                if i>1&j<N&I(i,j)==m-1 & I(i-1,j+1)==n-1
                    P(m,n,2)=P(m,n,2)+1;
                end
                if i<M & I(i,j)==m-1 & I(i+1,j)==n-1
                    P(m,n,3)=P(m,n,3)+1;
                end
                if i<M &j<N & I(i,j)==m-1 & I(i+1,j+1)==n-1
                    P(m,n,4)=P(m,n,4)+1;
                end
            end
        end
    end
end
for n=1:4
    P(:,:,n)=P(:,:,n)/sum(sum(P(:,:,n)));
end
H=zeros(1,4);
CON=H;
Ux=H;Uy=H;
deltaX=H; deltaY=H;
COR=H;
L=H;
for n=1:4
    ASM(n)=sum(sum(P(:,:,n).^2));
    for i=1:16
        for j=1:16
            if P(i,j,n)~=0
                H(n)=-P(i,j,n)*log(P(i,j,n))+H(n);
            end
            CON(n)=(i-j)^2*P(i,j,n)+CON(n);
            Ux(n)=i*P(i,j,n)+Ux(n);
            Uy(n)=j*P(i,j,n)+Ux(n);
        end
    end
end
for n=1:4
    for i=1:16
        for j=1:16
            deltaX(n)=(i-Ux(n))^2*P(i,j,n)+deltaX(n);
            deltaY(n)=(j-Uy(n))^2*P(i,j,n)+deltaY(n);
            COR(n)=i*j*P(i,j,n)+COR(n);
            L(n)=P(i,j,n)^2/(1+(i-j)^2)+L(n);
        end
    end
    COR(n)=(COR(n)-Ux(n)*Uy(n))/deltaX(n)/deltaY(n);
end
T=[ASM(1),ASM(2),ASM(3),ASM(4)];   %能量
t(1,:)=T;
U=[H(1),H(2),H(3),H(4)];           %熵
u(1,:)=U;
V=[CON(1),CON(2),CON(3),CON(4)];   %惯性矩
v(1,:)=V;
W=[COR(1),COR(2),COR(3),COR(4)];   %相关性
w(1,:)=W;
Z=[L(1),L(2),L(3),L(4)];           %逆差距
z(1,:)=Z; 


% --- Executes on button press in pushbutton39.
function pushbutton39_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton39 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                                       %取消警告
feature jit off                                                   %加速代码实现
f = handles.dd3;                                                   %种子分离后的RGB图像
rgb = f;                             %读取种子分离后的RGB图片
r = rgb(:,:,1);
g = rgb(:,:,2);
b = rgb(:,:,3);
R= medfilt2(r);
G = medfilt2(g);
B = medfilt2(b);
I= cat(3,R,G,B);                    %中值滤波处理
I=rgb2gray(I);
disp(I);
I=imresize(I,1/40);
[M,N]=size(I);
for i=1:M
    for j=1:N
        for n=1:16
            if(n-1)*16<=I(i,j)&I(i,j)<=(n-1)*16+15
                I(i,j)=n-1;
            end
        end
    end
end
P=zeros(16,16,4);
for m=1:16
    for n=1:16
        for i=1:M
            for j=1:N
                if j<N & I(i,j)==m-1 & I(i,j+1)==n-1
                    P(m,n,1)=P(m,n,1)+1;
                end
                if i>1&j<N&I(i,j)==m-1 & I(i-1,j+1)==n-1
                    P(m,n,2)=P(m,n,2)+1;
                end
                if i<M & I(i,j)==m-1 & I(i+1,j)==n-1
                    P(m,n,3)=P(m,n,3)+1;
                end
                if i<M &j<N & I(i,j)==m-1 & I(i+1,j+1)==n-1
                    P(m,n,4)=P(m,n,4)+1;
                end
            end
        end
    end
end
for n=1:4
    P(:,:,n)=P(:,:,n)/sum(sum(P(:,:,n)));
end
H=zeros(1,4);
CON=H;
Ux=H;Uy=H;
deltaX=H; deltaY=H;
COR=H;
L=H;
for n=1:4
    ASM(n)=sum(sum(P(:,:,n).^2));
    for i=1:16
        for j=1:16
            if P(i,j,n)~=0
                H(n)=-P(i,j,n)*log(P(i,j,n))+H(n);
            end
            CON(n)=(i-j)^2*P(i,j,n)+CON(n);
            Ux(n)=i*P(i,j,n)+Ux(n);
            Uy(n)=j*P(i,j,n)+Ux(n);
        end
    end
end
for n=1:4
    for i=1:16
        for j=1:16
            deltaX(n)=(i-Ux(n))^2*P(i,j,n)+deltaX(n);
            deltaY(n)=(j-Uy(n))^2*P(i,j,n)+deltaY(n);
            COR(n)=i*j*P(i,j,n)+COR(n);
            L(n)=P(i,j,n)^2/(1+(i-j)^2)+L(n);
        end
    end
    COR(n)=(COR(n)-Ux(n)*Uy(n))/deltaX(n)/deltaY(n);
end
T=[ASM(1),ASM(2),ASM(3),ASM(4)];   %能量
t(1,:)=T;
U=[H(1),H(2),H(3),H(4)];           %熵
u(1,:)=U;
V=[CON(1),CON(2),CON(3),CON(4)];   %惯性矩
v(1,:)=V;
W=[COR(1),COR(2),COR(3),COR(4)];   %相关性
w(1,:)=W;
Z=[L(1),L(2),L(3),L(4)];           %逆差距
z(1,:)=Z; 



% --- Executes on button press in pushbutton40.
function pushbutton40_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton40 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                                       %取消警告
feature jit off                                                   %加速代码实现
f = handles.dd4;                                                   %种子分离后的RGB图像
rgb = f;                             %读取种子分离后的RGB图片
r = rgb(:,:,1);
g = rgb(:,:,2);
b = rgb(:,:,3);
R= medfilt2(r);
G = medfilt2(g);
B = medfilt2(b);
I= cat(3,R,G,B);                    %中值滤波处理
I=rgb2gray(I);
disp(I);
I=imresize(I,1/40);
[M,N]=size(I);
for i=1:M
    for j=1:N
        for n=1:16
            if(n-1)*16<=I(i,j)&I(i,j)<=(n-1)*16+15
                I(i,j)=n-1;
            end
        end
    end
end
P=zeros(16,16,4);
for m=1:16
    for n=1:16
        for i=1:M
            for j=1:N
                if j<N & I(i,j)==m-1 & I(i,j+1)==n-1
                    P(m,n,1)=P(m,n,1)+1;
                end
                if i>1&j<N&I(i,j)==m-1 & I(i-1,j+1)==n-1
                    P(m,n,2)=P(m,n,2)+1;
                end
                if i<M & I(i,j)==m-1 & I(i+1,j)==n-1
                    P(m,n,3)=P(m,n,3)+1;
                end
                if i<M &j<N & I(i,j)==m-1 & I(i+1,j+1)==n-1
                    P(m,n,4)=P(m,n,4)+1;
                end
            end
        end
    end
end
for n=1:4
    P(:,:,n)=P(:,:,n)/sum(sum(P(:,:,n)));
end
H=zeros(1,4);
CON=H;
Ux=H;Uy=H;
deltaX=H; deltaY=H;
COR=H;
L=H;
for n=1:4
    ASM(n)=sum(sum(P(:,:,n).^2));
    for i=1:16
        for j=1:16
            if P(i,j,n)~=0
                H(n)=-P(i,j,n)*log(P(i,j,n))+H(n);
            end
            CON(n)=(i-j)^2*P(i,j,n)+CON(n);
            Ux(n)=i*P(i,j,n)+Ux(n);
            Uy(n)=j*P(i,j,n)+Ux(n);
        end
    end
end
for n=1:4
    for i=1:16
        for j=1:16
            deltaX(n)=(i-Ux(n))^2*P(i,j,n)+deltaX(n);
            deltaY(n)=(j-Uy(n))^2*P(i,j,n)+deltaY(n);
            COR(n)=i*j*P(i,j,n)+COR(n);
            L(n)=P(i,j,n)^2/(1+(i-j)^2)+L(n);
        end
    end
    COR(n)=(COR(n)-Ux(n)*Uy(n))/deltaX(n)/deltaY(n);
end
T=[ASM(1),ASM(2),ASM(3),ASM(4)];   %能量
t(1,:)=T;
U=[H(1),H(2),H(3),H(4)];           %熵
u(1,:)=U;
V=[CON(1),CON(2),CON(3),CON(4)];   %惯性矩
v(1,:)=V;
W=[COR(1),COR(2),COR(3),COR(4)];   %相关性
w(1,:)=W;
Z=[L(1),L(2),L(3),L(4)];           %逆差距
z(1,:)=Z; 




% --- Executes on button press in pushbutton41.
function pushbutton41_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                                       %取消警告
feature jit off                                                   %加速代码实现
f = handles.dd5;                                                   %种子分离后的RGB图像
rgb = f;                             %读取种子分离后的RGB图片
r = rgb(:,:,1);
g = rgb(:,:,2);
b = rgb(:,:,3);
R= medfilt2(r);
G = medfilt2(g);
B = medfilt2(b);
I= cat(3,R,G,B);                    %中值滤波处理
I=rgb2gray(I);
disp(I);
I=imresize(I,1/40);
[M,N]=size(I);
for i=1:M
    for j=1:N
        for n=1:16
            if(n-1)*16<=I(i,j)&I(i,j)<=(n-1)*16+15
                I(i,j)=n-1;
            end
        end
    end
end
P=zeros(16,16,4);
for m=1:16
    for n=1:16
        for i=1:M
            for j=1:N
                if j<N & I(i,j)==m-1 & I(i,j+1)==n-1
                    P(m,n,1)=P(m,n,1)+1;
                end
                if i>1&j<N&I(i,j)==m-1 & I(i-1,j+1)==n-1
                    P(m,n,2)=P(m,n,2)+1;
                end
                if i<M & I(i,j)==m-1 & I(i+1,j)==n-1
                    P(m,n,3)=P(m,n,3)+1;
                end
                if i<M &j<N & I(i,j)==m-1 & I(i+1,j+1)==n-1
                    P(m,n,4)=P(m,n,4)+1;
                end
            end
        end
    end
end
for n=1:4
    P(:,:,n)=P(:,:,n)/sum(sum(P(:,:,n)));
end
H=zeros(1,4);
CON=H;
Ux=H;Uy=H;
deltaX=H; deltaY=H;
COR=H;
L=H;
for n=1:4
    ASM(n)=sum(sum(P(:,:,n).^2));
    for i=1:16
        for j=1:16
            if P(i,j,n)~=0
                H(n)=-P(i,j,n)*log(P(i,j,n))+H(n);
            end
            CON(n)=(i-j)^2*P(i,j,n)+CON(n);
            Ux(n)=i*P(i,j,n)+Ux(n);
            Uy(n)=j*P(i,j,n)+Ux(n);
        end
    end
end
for n=1:4
    for i=1:16
        for j=1:16
            deltaX(n)=(i-Ux(n))^2*P(i,j,n)+deltaX(n);
            deltaY(n)=(j-Uy(n))^2*P(i,j,n)+deltaY(n);
            COR(n)=i*j*P(i,j,n)+COR(n);
            L(n)=P(i,j,n)^2/(1+(i-j)^2)+L(n);
        end
    end
    COR(n)=(COR(n)-Ux(n)*Uy(n))/deltaX(n)/deltaY(n);
end
T=[ASM(1),ASM(2),ASM(3),ASM(4)];   %能量
t(1,:)=T;
U=[H(1),H(2),H(3),H(4)];           %熵
u(1,:)=U;
V=[CON(1),CON(2),CON(3),CON(4)];   %惯性矩
v(1,:)=V;
W=[COR(1),COR(2),COR(3),COR(4)];   %相关性
w(1,:)=W;
Z=[L(1),L(2),L(3),L(4)];           %逆差距
z(1,:)=Z; 




% --- Executes on button press in pushbutton42.
function pushbutton42_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                                       %取消警告
feature jit off                                                   %加速代码实现
f = handles.dd6;                                                   %种子分离后的RGB图像
rgb = f;                             %读取种子分离后的RGB图片
r = rgb(:,:,1);
g = rgb(:,:,2);
b = rgb(:,:,3);
R= medfilt2(r);
G = medfilt2(g);
B = medfilt2(b);
I= cat(3,R,G,B);                    %中值滤波处理
I=rgb2gray(I);
disp(I);
I=imresize(I,1/40);
[M,N]=size(I);
for i=1:M
    for j=1:N
        for n=1:16
            if(n-1)*16<=I(i,j)&I(i,j)<=(n-1)*16+15
                I(i,j)=n-1;
            end
        end
    end
end
P=zeros(16,16,4);
for m=1:16
    for n=1:16
        for i=1:M
            for j=1:N
                if j<N & I(i,j)==m-1 & I(i,j+1)==n-1
                    P(m,n,1)=P(m,n,1)+1;
                end
                if i>1&j<N&I(i,j)==m-1 & I(i-1,j+1)==n-1
                    P(m,n,2)=P(m,n,2)+1;
                end
                if i<M & I(i,j)==m-1 & I(i+1,j)==n-1
                    P(m,n,3)=P(m,n,3)+1;
                end
                if i<M &j<N & I(i,j)==m-1 & I(i+1,j+1)==n-1
                    P(m,n,4)=P(m,n,4)+1;
                end
            end
        end
    end
end
for n=1:4
    P(:,:,n)=P(:,:,n)/sum(sum(P(:,:,n)));
end
H=zeros(1,4);
CON=H;
Ux=H;Uy=H;
deltaX=H; deltaY=H;
COR=H;
L=H;
for n=1:4
    ASM(n)=sum(sum(P(:,:,n).^2));
    for i=1:16
        for j=1:16
            if P(i,j,n)~=0
                H(n)=-P(i,j,n)*log(P(i,j,n))+H(n);
            end
            CON(n)=(i-j)^2*P(i,j,n)+CON(n);
            Ux(n)=i*P(i,j,n)+Ux(n);
            Uy(n)=j*P(i,j,n)+Ux(n);
        end
    end
end
for n=1:4
    for i=1:16
        for j=1:16
            deltaX(n)=(i-Ux(n))^2*P(i,j,n)+deltaX(n);
            deltaY(n)=(j-Uy(n))^2*P(i,j,n)+deltaY(n);
            COR(n)=i*j*P(i,j,n)+COR(n);
            L(n)=P(i,j,n)^2/(1+(i-j)^2)+L(n);
        end
    end
    COR(n)=(COR(n)-Ux(n)*Uy(n))/deltaX(n)/deltaY(n);
end
T=[ASM(1),ASM(2),ASM(3),ASM(4)];   %能量
t(1,:)=T;
U=[H(1),H(2),H(3),H(4)];           %熵
u(1,:)=U;
V=[CON(1),CON(2),CON(3),CON(4)];   %惯性矩
v(1,:)=V;
W=[COR(1),COR(2),COR(3),COR(4)];   %相关性
w(1,:)=W;
Z=[L(1),L(2),L(3),L(4)];           %逆差距
z(1,:)=Z; 





function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton44.
function pushbutton44_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                                       %取消警告
feature jit off                                                   %加速代码实现
% evalin('base','clear variables'); 
close(gcf)


% --- Executes on button press in pushbutton45.
function pushbutton45_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton45 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
L1 = handles.L1;
% assignin('base','A1',A1);        %为工作空间的变量指派值
L2 = handles.L2;
L3 = handles.L3;
L4 = handles.L4;
L5 = handles.L5;
L6 = handles.L6;
L = [L1 L2 L3 L4 L5 L6];
Lmean = mean(L);
% a = (A1+A2+A3+A4+A5+A6)/6;
assignin('base','Lmean',Lmean);        %为工作空间的变量指派值
a = num2str(Lmean);
set(handles.edit11,'String',a);
guidata(hObject,handles);

% --- Executes on button press in pushbutton46.
function pushbutton46_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton46 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
A1 = str2num(handles.total1);
% assignin('base','A1',A1);        %为工作空间的变量指派值
A2 = str2num(handles.total2);
A3 = str2num(handles.total3);
A4 = str2num(handles.total4);
A5 = str2num(handles.total5);
A6 = str2num(handles.total6);
A = [A1 A2 A3 A4 A5 A6];
Amean = mean(A);
% a = (A1+A2+A3+A4+A5+A6)/6;
assignin('base','Amean',Amean);        %为工作空间的变量指派值
a = num2str(Amean);
set(handles.edit9,'String',a);
guidata(hObject,handles);



% --- Executes on button press in pushbutton47.
function pushbutton47_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton47 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
A1 = str2num(handles.total1);
% assignin('base','A1',A1);        %为工作空间的变量指派值
A2 = str2num(handles.total2);
A3 = str2num(handles.total3);
A4 = str2num(handles.total4);
A5 = str2num(handles.total5);
A6 = str2num(handles.total6);
A = [A1 A2 A3 A4 A5 A6];
Astd = std(A);
Astd = num2str(Astd);
set(handles.edit10,'String',Astd);
guidata(hObject,handles);

% --- Executes on button press in pushbutton49.
function pushbutton49_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton49 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
L1 = handles.L1;
% assignin('base','A1',A1);        %为工作空间的变量指派值
L2 = handles.L2;
L3 = handles.L3;
L4 = handles.L4;
L5 = handles.L5;
L6 = handles.L6;
L = [L1 L2 L3 L4 L5 L6];
Lstd = std(L);
% a = (A1+A2+A3+A4+A5+A6)/6;
assignin('base','Lstd',Lstd);        %为工作空间的变量指派值
a = num2str(Lstd);
set(handles.edit13,'String',a);
guidata(hObject,handles);

% --- Executes on button press in pushbutton50.
function pushbutton50_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton50 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
S1 = handles.S1;
% assignin('base','A1',A1);        %为工作空间的变量指派值
S2 = handles.S2;
S3 = handles.S3;
S4 = handles.S4;
S5 = handles.S5;
S6 = handles.S6;
S = [S1 S2 S3 S4 S5 S6];
Smean = mean(S);
% a = (A1+A2+A3+A4+A5+A6)/6;
assignin('base','Smean',Smean);        %为工作空间的变量指派值
a = num2str(Smean);
set(handles.edit15,'String',a);
guidata(hObject,handles);

% --- Executes on button press in pushbutton51.
function pushbutton51_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton51 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
S1 = handles.S1;
% assignin('base','A1',A1);        %为工作空间的变量指派值
S2 = handles.S2;
S3 = handles.S3;
S4 = handles.S4;
S5 = handles.S5;
S6 = handles.S6;
S = [S1 S2 S3 S4 S5 S6];
Sstd = std(S);
% a = (A1+A2+A3+A4+A5+A6)/6;
assignin('base','Smean',Sstd);        %为工作空间的变量指派值
a = num2str(Sstd);
set(handles.edit14,'String',a);
guidata(hObject,handles);

% --- Executes on button press in pushbutton52.
function pushbutton52_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton52 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
R1 = handles.R1;
% assignin('base','A1',A1);        %为工作空间的变量指派值
R2 = handles.R2;
R3 = handles.R3;
R4 = handles.R4;
R5 = handles.R5;
R6 = handles.R6;
R = [R1 R2 R3 R4 R5 R6];
Rmean = mean(R);
% a = (A1+A2+A3+A4+A5+A6)/6;
assignin('base','Rmean',Rmean);        %为工作空间的变量指派值
a = num2str(Rmean);
set(handles.edit16,'String',a);
guidata(hObject,handles);

% --- Executes on button press in pushbutton53.
function pushbutton53_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton53 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
R1 = handles.R1;
% assignin('base','A1',A1);        %为工作空间的变量指派值
R2 = handles.R2;
R3 = handles.R3;
R4 = handles.R4;
R5 = handles.R5;
R6 = handles.R6;
R = [R1 R2 R3 R4 R5 R6];
Rstd = std(R);
% a = (A1+A2+A3+A4+A5+A6)/6;
assignin('base','Rstd',Rstd);        %为工作空间的变量指派值
a = num2str(Rstd);
set(handles.edit17,'String',a);
guidata(hObject,handles);


function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit15_Callback(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double


% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit16_Callback(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double


% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit17_Callback(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit17 as text
%        str2double(get(hObject,'String')) returns contents of edit17 as a double


% --- Executes during object creation, after setting all properties.
function edit17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function pushbutton46_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton46 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function pushbutton45_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton45 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in pushbutton54.
function pushbutton54_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton54 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

warning off                                      %取消警告
feature jit off                                  %加速代码实现 
L = handles.L1;
S = handles.S1;
R = L/S;
% assignin('base','Lmean',R);        %为工作空间的变量指派值
a = num2str(R);
set(handles.text20,'String',a);
handles.R1 = R;
guidata(hObject,handles);

% --- Executes on button press in pushbutton55.
function pushbutton55_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton55 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
L = handles.L2;
S = handles.S2;
R = L/S;
% assignin('base','Lmean',R);        %为工作空间的变量指派值
a = num2str(R);
set(handles.text21,'String',a);
handles.R2 = R;
guidata(hObject,handles);

% --- Executes on button press in pushbutton56.
function pushbutton56_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton56 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
L = handles.L3;
S = handles.S3;
R = L/S;
% assignin('base','Lmean',R);        %为工作空间的变量指派值
a = num2str(R);
set(handles.text22,'String',a);
handles.R3 = R;
guidata(hObject,handles);

% --- Executes on button press in pushbutton57.
function pushbutton57_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton57 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
L = handles.L4;
S = handles.S4;
R = L/S;
% assignin('base','Lmean',R);        %为工作空间的变量指派值
a = num2str(R);
set(handles.text23,'String',a);
handles.R4 = R;
guidata(hObject,handles);

% --- Executes on button press in pushbutton58.
function pushbutton58_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton58 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
L = handles.L5;
S = handles.S5;
R = L/S;
% assignin('base','Lmean',R);        %为工作空间的变量指派值
a = num2str(R);
set(handles.text24,'String',a);
handles.R5 = R;
guidata(hObject,handles);

% --- Executes on button press in pushbutton59.
function pushbutton59_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton59 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现 
L = handles.L6;
S = handles.S6;
R = L/S;
% assignin('base','Lmean',R);        %为工作空间的变量指派值
a = num2str(R);
set(handles.text25,'String',a);
handles.R6 = R;
guidata(hObject,handles);




% --- Executes on button press in pushbutton61.
function pushbutton61_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton61 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现     
z=importdata('mat1.mat');                      %全部种子阈值分割后的二值图像                                                  
% grayimg = rgb2gray(z);                       %将原图像灰度化
f2=im2double(z);                               %将灰度图像转换为双精度型图像 
h=graythresh(f2);                              %自动阈值分割
K=im2bw(f2,h); 
K=imfill(K,'holes');
f3=K;
%先闭运算 再开运算
se=strel('disk',5);
L = imclose(f3,se);                                                  %对小麦种子进行开运算
f4 = imopen(L,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(f4,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
%M是大小和BWing一样的图像数组，里面存放着对bwing图像的标签值（即判定为连通后，在L矩阵中标记出来） 
% figure('NumberTitle','off','Name','Regional marking of wheat seeds'),      %小麦种子的区域标记
% subplot(121),imshow(f4);title('Morphologically processed image')           %显示形态学操作后的图像
% subplot(122),imshow(L);title('Tagged image')                               %显示标记后的图像  

global status N5
status=regionprops(L,'BoundingBox');
% centroid=regionprops(L,'Centroid');
% %返回值STATS是一个长度为max(L(:))的结构数组，结构数组的相应域定义了每一个区域相应属性下的度量
% for i=1:num
%     rectangle('position',status(i).BoundingBox,'edgecolor','r');
% %参数说明：position绘制的为二维图像（他是通过对角的两点确定矩形框）
% %edgecolor 指边缘图像，r表示变换为红色。
% %facecolor 指内部填充颜色。
%     text(centroid(i,1).Centroid(1,1)-10,centroid(i,1).Centroid(1,2)-20, num2str(i),'Color', 'r')
% %这个是为绘制出来的矩形框图标记数字
% end
copy_L=L;
L_part5=(copy_L==N5);                                            %进行区域的选择，此处只保留2
% image_part1 = (mark_image ~= 1);
axes(handles.axes5);
imshow(L_part5);title('Separated seed binary image');              %显示分离后的二值图像
handles.d5=L_part5;                                             %分离第六个小麦的二值图像
guidata(hObject,handles);


% --- Executes on button press in pushbutton62.
function pushbutton62_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton62 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现
set(handles.text2,'string','');
set(handles.text3,'string','');
set(handles.text4,'string','');
set(handles.text5,'string','');
set(handles.text6,'string','');
set(handles.text7,'string','');
set(handles.text8,'string','');
set(handles.text9,'string','');
set(handles.text10,'string','');
set(handles.text11,'string','');
set(handles.text12,'string','');
set(handles.text13,'string','');
set(handles.text14,'string','');
set(handles.text15,'string','');
set(handles.text16,'string','');
set(handles.text17,'string','');
set(handles.text18,'string','');
set(handles.text19,'string','');
set(handles.text20,'string','');
set(handles.text21,'string','');
set(handles.text22,'string','');
set(handles.text23,'string','');
set(handles.text24,'string','');
set(handles.text25,'string','');
guidata(hObject,handles);


% --- Executes on button press in pushbutton63.
function pushbutton63_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton63 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现    
f1=handles.d5;                                   %分离第四个小麦的二值图像
f2 = handles.f3;                                 %分离的硬币二值图像
total=bwarea(f1);                                 %小麦的像素面积
total2=bwarea(f2);                               %硬币的像素面积
c = 551.266;                                     %硬币的实际面积
p = (c*total)/total2;                            %小麦的实际面积                       
totalp=num2str(p);
set(handles.text14,'string',totalp);
handles.total4=totalp;
guidata(hObject,handles);

% --- Executes on button press in pushbutton64.
function pushbutton64_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton64 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
f=handles.d5;                                     %分离第四个小麦的二值图像
f1 = handles.f3;                                 %分离的硬币二值图像
L=logical(f);                                    
L1 = logical(f1);                                
format long;
a=regionprops(L,'MajorAxisLength');             %计算小麦像素的长颈
a1 = regionprops(L1,'MajorAxisLength');         %计算硬币像素的直径
a=struct2cell(a);
a1 = struct2cell(a1);
a=cell2mat(a);
a1 = cell2mat(a1);
c = 25;                                        %硬币的实际直径
p = (c*a)/a1;                                  %小麦的实际长颈
% f=texth=cell2mat(h)
% wrap(g,h);
% a1=a(1),a2=a(2),a3=a(3),a4=a(4)
% b1=b(1),b2=b(2),b3=b(3),b4=b(4)
set(handles.text15,'string',p);
handles.L5 = p;
guidata(hObject,handles);

% --- Executes on button press in pushbutton65.
function pushbutton65_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton65 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
f=handles.d5;                                     %分离第四个小麦的二值图像
f1 = handles.f3;                                 %分离的硬币二值图像
S=logical(f);
S1=logical(f1);
format long;
b=regionprops(S,'MinorAxisLength');              %计算小麦像素短颈
b1 = regionprops(S1,'MinorAxisLength');          %计算硬币像素直径
b=struct2cell(b);
b1 = struct2cell(b1);
% g = uicontrol('style', 'text', 'position', [20 20 100 100]);
b=cell2mat(b);
b1 = cell2mat(b1);
c = 25;
p = (c*b)/b1;
% f=texth=cell2mat(h)
% wrap(g,h);
% a1=a(1),a2=a(2),a3=a(3),a4=a(4)
% b1=b(1),b2=b(2),b3=b(3),b4=b(4)
set(handles.text16,'string',p);
handles.S5 = p;
guidata(hObject,handles);


% --- Executes on button press in pushbutton66.
function pushbutton66_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton66 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
f=handles.d4;
[J, thresh]=edge(f, 'sobel', [], 'horizontal');                      %使用sobel算子进行边缘提取
[K, thresh]=edge(f, 'canny');                                       %使用canny算子进行边缘提取
[S, thresh]=edge(f, 'log', [], 2.3);                                %使用log算子进行边缘提取 
% H=bwperim(f,8);                                                   %采用函数bwperim获取二值图像的边缘
H=bwmorph(f,'remove');                                              %移除二值图像内部的像素点 
se=strel('disk', 2);                                                 %先膨胀后腐蚀进行边缘提取
M=imdilate(f, se);
N=imerode(f, se);
L=M-N;
figure('NumberTitle','off','Name','Edge extraction of the fourth wheat seed'),        %第三粒小麦种子的边缘提取
subplot(221);  imshow(J);title('Image using sobel operator for edge extraction')      %显示使用sobel算子进行边缘提取的图像
subplot(222);  imshow(K);title('Image using canny operator for edge extraction')      %显示使用canny算子进行边缘提取的图像
subplot(223);  imshow(S);title('Image using log operator for edge extraction')        %显示使用log算子进行边缘提取的图像
subplot(224);  imshow(L);title('Dilate and erode the image for edge extraction')      %显示先膨胀后腐蚀进行边缘提取的图像
handles.L4=L;
guidata(hObject,handles);
