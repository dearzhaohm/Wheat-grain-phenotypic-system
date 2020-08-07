 function varargout = Subsystem_xiugai1(varargin)
% Subsystem_xiugai1 MATLAB code for Subsystem_xiugai1.fig
%      Subsystem_xiugai1, by itself, creates a new Subsystem_xiugai1 or raises the existing
%      singleton*.
%
%      H = Subsystem_xiugai1 returns the handle to a new Subsystem_xiugai1 or the handle to
%      the existing singleton*.
%
%      Subsystem_xiugai1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in Subsystem_xiugai1.M with the given input arguments.
%
%      Subsystem_xiugai1('Property','Value',...) creates a new Subsystem_xiugai1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Subsystem_xiugai1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Subsystem_xiugai1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Subsystem_xiugai1

% Last Modified by GUIDE v2.5 24-Jul-2020 12:22:59

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Subsystem_xiugai1_OpeningFcn, ...
                   'gui_OutputFcn',  @Subsystem_xiugai1_OutputFcn, ...
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


% --- Executes just before Subsystem_xiugai1 is made visible.
function Subsystem_xiugai1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Subsystem_xiugai1 (see VARARGIN)

% Choose default command line output for Subsystem_xiugai1
handles.output = hObject;
global clickcounts
clickcounts = 0;
set(gcf,'name','Subsystem Color analysis'); %第三个参数为要修改的界面名称
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Subsystem_xiugai1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Subsystem_xiugai1_OutputFcn(hObject, eventdata, handles) 
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
imshow(L_part2);title('Binary image after separation of wheat seeds');               %显示分离后的二值图像
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







% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
global status  N1;
% clickcounts = clickcounts+1;
% set(hObject,'string',num2str(clickcounts));
axes(handles.axes1);
I=handles.f1;                                     %全部小麦种子原始图像
I1=handles.f;                                     %分离第一个小麦的二值图像

for i=1:3
    I2(:,:,i)=I(:,:,i).*uint8(I1);
end

a = struct2cell(status);
x = a(N1);
x = cell2mat(x);
x1 = x(1);
y1 = x(2);
w = x(3);
h = x(4);
f1 = imcrop(I2,[x1,y1,125,125]);
imshow(f1);title('Seed remove background RGB image')                  %种子去背景的RGB图像
h=ones(3,3)/9;                                                        %线性滤波
K=imfilter(f1,h);
% figure,
% subplot(121);imshow(I2);title('原始图像')                            %显示原始图像
% subplot(122);imshow(K);title('线性滤波后的图像')                     %显示线性滤波后的图像
R=f1(:,:,1);                                      %原图像的R分量
G=f1(:,:,2);                                      %原图像的G分量
B=f1(:,:,3);                                      %原图像的B分量 
Y = R * 0.299 + G * 0.587 + B * 0.114;
R1=R(:);
G1=G(:);
B1=B(:);
Y1=Y(:);
Rmax=max(R1);
Gmax=max(G1);
Bmax=max(B1);
Ymax=max(Y1);
bR = 5:255;
bG = 5:255;
bB = 5:255;
bY = 5:255;
cR = histc(R1,bR);
cG = histc(G1,bG);
cB = histc(B1,bB);
cY = histc(Y1,bY);
[max_num, max_indexR] = max(cR);
[max_num2, max_indexG] = max(cG);
[max_num3, max_indexB] = max(cB);
[max_num4, max_indexY] = max(cY);
max_indexR=max_indexR+4;
max_indexG=max_indexG+4;
max_indexB=max_indexB+4;
max_indexY=max_indexY+4;
disp(max_indexY);
% Ymax = max(max(Y));
% Rmax = max(max(R));
% Gmax = max(max(G));
% Bmax = max(max(B));
M = [max_indexR,max_indexG,max_indexB];
% YUVimag = rgb2ycbcr(f1);
% Y = YUVimag(:,:,1)
% figure,
% imshow(Y);
% MAX = max(M) 
%the num should be a 1x3 Integer mat limited in [0 255]
exchange_list={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
string='#';
for i=1:3
    temp_num=M(i);
    string(1+i*2-1)=exchange_list{(temp_num-mod(temp_num,16))/16+1};
    string(1+i*2)=exchange_list{mod(temp_num,16)+1};
end
  
HTML = string;
MAX = max(HTML);
% figure('NumberTitle','off','Name','RGB image and histogram after separation of wheat seeds'),%小麦种子分离后的RGB图像和直方图
% 
% subplot(4,2,1),imshow(f1),title('RGB image after separation')                                            %显示小麦种子分离后的RGB图像
% subplot(4,2,2),imhist(f1),title('Histogram of RGB image after separation') %显示小麦种子分离后RGB图像的直方图
% % axis([0,255,0,2000]);
% subplot(4,2,3),imshow(R),title('Grayscale image of R component after separation')                        %显示小麦种子分离后R分量灰度图
% subplot(4,2,4),imhist(R),title('Histogram at red resolution after separation')%显示小麦种子分离后绿色分辨率下的直方图
% % axis([0,255,0,2000]);
% subplot(4,2,5),imshow(G),title('G-component grayscale image after separation')                           %显示小麦种子分离后G分量灰度图
% subplot(4,2,6),imhist(G),title('Histogram at green resolution after separation')%显示小麦种子分离后绿色分辨率下的直方图
% % axis([0,255,0,2000]);
% subplot(4,2,7),imshow(B),title('G-component grayscale image after separation')                           %显示小麦种子分离后G分量灰度图
% subplot(4,2,8),imhist(B),title('Histogram at blue resolution after separation')%显示小麦种子分离后蓝色分辨率下的直方图
% % axis([0,255,0,2000]);
% hold on
% 
% %set(gca,'YLim',[0 2000]);
% xlim=get(gca,'xlim');
% ylim=get(gca,'ylim');
% % title('First wheat seed','position',[100,140],'FontSize',12);
% 
% N=160;
% text(sum(xlim)/2-(1.42*N),sum(ylim)/2-(52*N),'First wheat seed','horiz','center','FontSize',16,'FontWeight','Bold')      %第一粒小麦种子
% 
% imwrite(frame,['D:\Seed discrimination\image_1\',num2str(clickcounts),'.jpg']);
% 
% R=double(R);G=double(G);B=double(B);
% hsv=rgb2hsv(f1);                                              %图像由rgb颜色空间转换到hsv颜色空间
% h=hsv(:,:,1);                                                %为色调h赋值                                   
% s=hsv(:,:,2);                                                %为饱和度s赋值
% v=hsv(:,:,3);                                                %为亮度v赋值                                                                                                          
% figure('NumberTitle','off','Name','Hsv color space image and histogram after separation of wheat seeds'),  %小麦种子分离后的hsv颜色空间图像和直方图
% subplot(4,2,1),imshow(hsv),title('Hsv color space image after separation')                                 %显示小麦种子分离后的hsv颜色空间图像
% subplot(4,2,2),imhist(hsv),title('Hsv space image histogram after separation')                              %显示小麦种子图像分离后hsv颜色空间的直方图
% subplot(4,2,3),imshow(h),title('Grayscale image with hue h after separation')                               %显示小麦种子分离后色调h的灰度图
% subplot(4,2,4),imhist(h),title('Histogram based on hue h after separation')                                 %显示小麦种子分离后基于色调h的直方图
% subplot(4,2,5),imshow(s),title('Gray image of the saturation s after separation')                           %显示小麦种子分离后饱和度s的灰度图
% subplot(4,2,6),imhist(s),title('Histogram based on saturation s after separation')                          %显示小麦种子分离后基于饱和度s的直方图
% subplot(4,2,7),imshow(v),title('Gray image of brightness v after separation')                               %显示小麦种子分离后亮度v的灰度图
% subplot(4,2,8),imhist(v),title('Histogram based on brightness v after separation')                          %显示小麦种子分离后基于亮度v的直方图
% hold on
% %set(gca,'YLim',[0 2000]);
% xlim=get(gca,'xlim');
% ylim=get(gca,'ylim');
% N=200;
% text(sum(xlim)/2-(N/240),sum(ylim)/2-(42*N),'First wheat seed','horiz','center','FontSize',16,'FontWeight','Bold')    %第一粒小麦种子
axes(handles.axes7);
imhist(R),title('R-Histogram ') %显示小麦种子分离后绿色分辨率下的直方图
axes(handles.axes8);
imhist(G),title('G-component ')                           %显示小麦种子分离后G分量灰度图
axes(handles.axes9);
imhist(B),title('B-Histogram ')                          %显示小麦种子分离后蓝色分辨率下的直方图
[m,n]=size(f1);                             %求图像f1数据矩阵的大小赋值给[m,n],表示m×n维矩阵
mm=round(m/2);                              %对m/2取整赋值给mm
mn=round(n/2);                             
[p,q]=size(K);                             
pp=round(p/2);
qq=round(q/2);
f1=double(f1);                               %图像数据变为double型
% K=double(K);
colorsum=0.0;                              %给灰度值总和赋0值
Iavg1=mean2(f1);
Havg1=mean2(hsv);
f1=double(f1);
H=double(hsv); 
Ravg1=mean2(R);         %R分量的均值
Gavg1=mean2(G);         %G分量的均值
Bavg1=mean2(B);         %B分量的均值
Iavg=mean2(f1);          %求原图像一阶矩
R = double(R);
G = double(G);
B = double(B);

Rstd1=std(std(R));      %R分量的方差
Gstd1=std(std(G));      %G分量的方差
Bstd1=std(std(B));      %B分量的方差
Istd=std(std(f1));      %求原图像二阶矩

for i=1:mm                                 %循环求解灰度值总和   
    for j=1:mn
        colorsum=colorsum+(f1(i,j)-Iavg)^3;
    end
end

Iske=(colorsum/(mm*mn)).^(1/3);                %求种子分离后RGB图像三阶矩 

% havg1=mean2(h)       %求h分量的灰度均值
% savg1=mean2(s)       %求s分量的灰度均值
% vavg1=mean2(v)       %求v分量的灰度均值
% hstd1=std(std(h))    %h分量的方差 
% sstd1=std(std(s))    %s分量的方差 
% vstd1=std(std(v))    %v分量的方差 
handles.Y1 = max_indexY;
handles.M1 = M;
handles.f2 = f1;                                     %小麦种子分离后的RGB图像
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
z=handles.er;                                   %全部种子阈值分割后的二值图像                                                  
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
imshow(L_part2);title('Binary image after separation of wheat seeds');              %显示分离后的二值图像
handles.d2=L_part2;                                                                 %分离第二个小麦的二值图像
guidata(hObject,handles);









% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
global status  N2;

axes(handles.axes2);
I=handles.f1;                                     %全部小麦种子原始图像
I1=handles.d2;                                    %分离第二个小麦的二值图像
for i=1:3
    I2(:,:,i)=I(:,:,i).*uint8(I1);
end
a = struct2cell(status);
x = a(N2);
x = cell2mat(x);
x1 = x(1);
y1 = x(2);
f1 = imcrop(I2,[x1,y1,125,125]);
imshow(f1);title('Seed remove background RGB image')

h=ones(3,3)/9;                                   %线性滤波
K=imfilter(f1,h);
% figure,
% subplot(121);imshow(I2);title('原始图像')                         %显示原始图像
% subplot(122);imshow(K);title('线性滤波后的图像')                   %显示线性滤波后的图像
R=f1(:,:,1);                                      %原图像的R分量
G=f1(:,:,2);                                      %原图像的G分量
B=f1(:,:,3);                                      %原图像的B分量 
Y = R * 0.299 + G * 0.587 + B * 0.114;
% Ymax = max(max(Y));
% Rmax = max(max(R));
% Gmax = max(max(G));
% Bmax = max(max(B));
R1=R(:);
G1=G(:);
B1=B(:);
Y1=Y(:);
Rmax=max(R1);
Gmax=max(G1);
Bmax=max(B1);
Ymax=max(Y1);
bR = 5:255;
bG = 5:255;
bB = 5:255;
bY = 5:255;
cR = histc(R1,bR);
cG = histc(G1,bG);
cB = histc(B1,bB);
cY = histc(Y1,bY);
[max_num, max_indexR] = max(cR);
[max_num2, max_indexG] = max(cG);
[max_num3, max_indexB] = max(cB);
[max_num4, max_indexY] = max(cY);
max_indexR=max_indexR+4;
max_indexG=max_indexG+4;
max_indexB=max_indexB+4;
max_indexY=max_indexY+4;
disp(max_indexY);
M = [max_indexR,max_indexG,max_indexB];

% M = [Rmax,Gmax,Bmax];
% YUVimag = rgb2ycbcr(f1);
% Y = YUVimag(:,:,1)
% figure,
% imshow(Y);
% MAX = max(M) 
%the num should be a 1x3 Integer mat limited in [0 255]
exchange_list={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
string='#';
for i=1:3
    temp_num=M(i);
    string(1+i*2-1)=exchange_list{(temp_num-mod(temp_num,16))/16+1};
    string(1+i*2)=exchange_list{mod(temp_num,16)+1};
end
  
HTML = string;
MAX = max(HTML);

axes(handles.axes10);
imhist(R),title('R-Histogram ') %显示小麦种子分离后绿色分辨率下的直方图
axes(handles.axes11);
imhist(G),title('G-component ')                           %显示小麦种子分离后G分量灰度图
axes(handles.axes12);
imhist(B),title('B-Histogram ')                          %显示小麦种子分离后蓝色分辨率下的直方图

[m,n]=size(f1);                             %求图像f1数据矩阵的大小赋值给[m,n],表示m×n维矩阵
mm=round(m/2);                              %对m/2取整赋值给mm
mn=round(n/2);                             
[p,q]=size(K);                             
pp=round(p/2);
qq=round(q/2);
f1=double(f1);                               %图像数据变为double型
% K=double(K);
colorsum=0.0;                              %给灰度值总和赋0值
Iavg1=mean2(f1);
Havg1=mean2(hsv);
f1=double(f1);
H=double(hsv); 
Ravg1=mean2(R);         %R分量的均值
Gavg1=mean2(G);         %G分量的均值
Bavg1=mean2(B);         %B分量的均值
Iavg=mean2(f1);          %求原图像一阶矩
R = double(R);
G = double(G);
B = double(B);
Rstd1=std(std(R));      %R分量的方差
Gstd1=std(std(G));      %G分量的方差
Bstd1=std(std(B));      %B分量的方差
Istd=std(std(f1));      %求原图像二阶矩

for i=1:mm                                 %循环求解灰度值总和   
    for j=1:mn
        colorsum=colorsum+(f1(i,j)-Iavg)^3;
    end
end

Iske=(colorsum/(mm*mn))^(1/3);                %求种子分离后RGB图像三阶矩 

% havg1=mean2(h)       %求h分量的灰度均值
% savg1=mean2(s)       %求s分量的灰度均值
% vavg1=mean2(v)       %求v分量的灰度均值
% hstd1=std(std(h))    %h分量的方差 
% sstd1=std(std(s))    %s分量的方差 
% vstd1=std(std(v))    %v分量的方差 
handles.Y2 = max_indexY;
handles.M2 = M;
handles.dd2 = f1;                                     %小麦种子分离后的RGB图像
guidata(hObject,handles);



% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现     
z=handles.er;                                   %全部种子阈值分割后的二值图像                                                  
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
imshow(L_part3);title('Binary image of Separated wheat seed');              %显示分离后的二值图像
handles.d3=L_part3;                                                                 %分离第三个小麦的二值图像
guidata(hObject,handles);







% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
global status  N3;
% clickcounts = clickcounts+1;
% set(hObject,'string',num2str(clickcounts));
axes(handles.axes3);
I=handles.f1;                                     %全部小麦种子原始图像
I1=handles.d3;                                    %分离第二个小麦的二值图像
for i=1:3
    I2(:,:,i)=I(:,:,i).*uint8(I1);
end
a = struct2cell(status);
x = a(N3);
x = cell2mat(x);
x1 = x(1);
y1 = x(2);
f1 = imcrop(I2,[x1,y1,125,125]);
imshow(f1);title('Seed remove background RGB image')
frame = f1;
h=ones(3,3)/9;                                   %线性滤波
K=imfilter(f1,h);
% figure,
% subplot(121);imshow(I2);title('原始图像')                         %显示原始图像
% subplot(122);imshow(K);title('线性滤波后的图像')                   %显示线性滤波后的图像
R=f1(:,:,1);                                      %原图像的R分量
G=f1(:,:,2);                                      %原图像的G分量
B=f1(:,:,3);                                      %原图像的B分量 
Y = R * 0.299 + G * 0.587 + B * 0.114;
R1=R(:);
G1=G(:);
B1=B(:);
Y1=Y(:);
Rmax=max(R1);
Gmax=max(G1);
Bmax=max(B1);
Ymax=max(Y1);
bR = 5:255;
bG = 5:255;
bB = 5:255;
bY = 5:255;
cR = histc(R1,bR);
cG = histc(G1,bG);
cB = histc(B1,bB);
cY = histc(Y1,bY);
[max_num, max_indexR] = max(cR);
[max_num2, max_indexG] = max(cG);
[max_num3, max_indexB] = max(cB);
[max_num4, max_indexY] = max(cY);
max_indexR=max_indexR+4;
max_indexG=max_indexG+4;
max_indexB=max_indexB+4;
max_indexY=max_indexY+4;

M = [max_indexR,max_indexG,max_indexB];
% Yceshi=Y(:);
% maxY=max(Yceshi);
% bY = 5:255;
% cY = histc(Yceshi,bY);
% [max_num4, Y_index] = max(cY);
% Y_index=Y_index+4;
% Ymax = max(max(Y));
% Rmax = max(max(R));
% Gmax = max(max(G));
% Bmax = max(max(B));
% M = [Rmax,Gmax,Bmax];
% YUVimag = rgb2ycbcr(f1);
% Y = YUVimag(:,:,1)
% figure,
% imshow(Y);
% MAX = max(M) 
%the num should be a 1x3 Integer mat limited in [0 255]
exchange_list={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
string='#';
for i=1:3
    temp_num=M(i);
    string(1+i*2-1)=exchange_list{(temp_num-mod(temp_num,16))/16+1};
    string(1+i*2)=exchange_list{mod(temp_num,16)+1};
end
  
HTML = string;
MAX = max(HTML);

axes(handles.axes13);
imhist(R),title('R-Histogram ') %显示小麦种子分离后绿色分辨率下的直方图
axes(handles.axes14);
imhist(G),title('G-component ')                           %显示小麦种子分离后G分量灰度图
axes(handles.axes15);
imhist(B),title('B-Histogram ')                          %显示小麦种子分离后蓝色分辨率下的直方图
% figure('NumberTitle','off','Name','RGB image and histogram after separation of wheat seeds'),            %小麦种子分离后的RGB图像和直方图
% subplot(4,2,1),imshow(f1),title('RGB image after separation')                                            %显示小麦种子分离后的RGB图像
% subplot(4,2,2),imhist(f1),title('Histogram of RGB image after separation')                               %显示小麦种子分离后RGB图像的直方图
% subplot(4,2,3),imshow(R),title('Grayscale image of R component after separation')                        %显示小麦种子分离后R分量灰度图
% subplot(4,2,4),imhist(R),title('Histogram at red resolution after separation')                           %显示小麦种子分离后绿色分辨率下的直方图
% subplot(4,2,5),imshow(G),title('G-component grayscale image after separation')                           %显示小麦种子分离后G分量灰度图
% subplot(4,2,6),imhist(G),title('Histogram at green resolution after separation')                         %显示小麦种子分离后绿色分辨率下的直方图
% subplot(4,2,7),imshow(B),title('G-component grayscale image after separation')                           %显示小麦种子分离后G分量灰度图
% subplot(4,2,8),imhist(B),title('Histogram at blue resolution after separation')                          %显示小麦种子分离后蓝色分辨率下的直方图
% hold on
% %set(gca,'YLim',[0 2000]);
% xlim=get(gca,'xlim');
% ylim=get(gca,'ylim');
% N=160;
% text(sum(xlim)/2-(1.42*N),sum(ylim)/2-(55*N),'Third wheat seed','horiz','center','FontSize',16,'FontWeight','Bold')
% 
% imwrite(frame,['D:\Seed discrimination\image_1\',num2str(clickcounts),'.jpg']);
% 
% R=double(R);G=double(G);B=double(B);
% hsv=rgb2hsv(f1);                                             %图像由rgb颜色空间转换到hsv颜色空间
% h=hsv(:,:,1);                                                %为色调h赋值                                   
% s=hsv(:,:,2);                                                %为饱和度s赋值
% v=hsv(:,:,3);                                                %为亮度v赋值                                                                       
%                                    
% figure('NumberTitle','off','Name','Hsv color space image and histogram after separation of wheat seeds'),  %小麦种子分离后的hsv颜色空间图像和直方图
% subplot(4,2,1),imshow(hsv),title('Hsv color space image after separation')                                 %显示小麦种子分离后的hsv颜色空间图像
% subplot(4,2,2),imhist(hsv),title('Hsv space image histogram after separation')                              %显示小麦种子图像分离后hsv颜色空间的直方图
% subplot(4,2,3),imshow(h),title('Grayscale image with hue h after separation')                               %显示小麦种子分离后色调h的灰度图
% subplot(4,2,4),imhist(h),title('Histogram based on hue h after separation')                                 %显示小麦种子分离后基于色调h的直方图
% subplot(4,2,5),imshow(s),title('Gray image of the saturation s after separation')                           %显示小麦种子分离后饱和度s的灰度图
% subplot(4,2,6),imhist(s),title('Histogram based on saturation s after separation')                          %显示小麦种子分离后基于饱和度s的直方图
% subplot(4,2,7),imshow(v),title('Gray image of brightness v after separation')                               %显示小麦种子分离后亮度v的灰度图
% subplot(4,2,8),imhist(v),title('Histogram based on brightness v after separation')                          %显示小麦种子分离后基于亮度v的直方图
% hold on
% %set(gca,'YLim',[0 2000]);
% xlim=get(gca,'xlim');
% ylim=get(gca,'ylim');
% N=200;
% text(sum(xlim)/2-(N/240),sum(ylim)/2-(45*N),'Third wheat seed','horiz','center','FontSize',16,'FontWeight','Bold')

[m,n]=size(f1);                             %求图像f1数据矩阵的大小赋值给[m,n],表示m×n维矩阵
mm=round(m/2);                              %对m/2取整赋值给mm
mn=round(n/2);                             
[p,q]=size(K);                             
pp=round(p/2);
qq=round(q/2);
f1=double(f1);                               %图像数据变为double型
% K=double(K);
colorsum=0.0;                              %给灰度值总和赋0值
Iavg1=mean2(f1);
Havg1=mean2(hsv);

H=double(hsv); 
Ravg1=mean2(R);         %R分量的均值
Gavg1=mean2(G);         %G分量的均值
Bavg1=mean2(B);         %B分量的均值
Iavg=mean2(f1);          %求原图像一阶矩
R = double(R);
G = double(G);
B = double(B);
Rstd1=std(std(R));      %R分量的方差
Gstd1=std(std(G));      %G分量的方差
Bstd1=std(std(B));      %B分量的方差
Istd=std(std(f1));     %求原图像二阶矩

for i=1:mm                                 %循环求解灰度值总和   
    for j=1:mn
        colorsum=colorsum+(f1(i,j)-Iavg)^3;
    end
end

Iske=(colorsum/(mm*mn))^(1/3);                %求种子分离后RGB图像三阶矩 

% havg1=mean2(h)       %求h分量的灰度均值
% savg1=mean2(s)       %求s分量的灰度均值
% vavg1=mean2(v)       %求v分量的灰度均值
% hstd1=std(std(h))    %h分量的方差 
% sstd1=std(std(s))    %s分量的方差 
% vstd1=std(std(v))    %v分量的方差 
handles.Y3 = max_indexY;
handles.M3 = M;
handles.dd3 = f1;                                     %小麦种子分离后的RGB图像
guidata(hObject,handles);







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
z=handles.er;                                   %全部种子阈值分割后的二值图像                                                  
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
imshow(L_part4);title('Binary image after separation of wheat seeds');              %显示分离后的二值图像
handles.d4=L_part4;                                             %分离第四个小麦的二值图像
guidata(hObject,handles);










% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
global status  N4;
% clickcounts = clickcounts+1;
% set(hObject,'string',num2str(clickcounts));
axes(handles.axes4);
I=handles.f1;                                     %全部小麦种子原始图像
I1=handles.d4;                                    %分离第四个小麦的二值图像
for i=1:3
    I2(:,:,i)=I(:,:,i).*uint8(I1);
end
a = struct2cell(status);
x = a(N4);
x = cell2mat(x);
x1 = x(1);
y1 = x(2);
f1 = imcrop(I2,[x1,y1,125,125]);
imshow(f1);title('Seed remove background RGB image')
frame = f1;
h=ones(3,3)/9;                                   %线性滤波
K=imfilter(f1,h);
% figure,
% subplot(121);imshow(I2);title('原始图像')                         %显示原始图像
% subplot(122);imshow(K);title('线性滤波后的图像')                   %显示线性滤波后的图像
R=f1(:,:,1);                                      %原图像的R分量
G=f1(:,:,2);                                      %原图像的G分量
B=f1(:,:,3);                                      %原图像的B分量 
Y = R * 0.299 + G * 0.587 + B * 0.114;
% Ymax = max(max(Y));
% Rmax = max(max(R));
% Gmax = max(max(G));
% Bmax = max(max(B));
R1=R(:);
G1=G(:);
B1=B(:);
Y1=Y(:);
Rmax=max(R1);
Gmax=max(G1);
Bmax=max(B1);
Ymax=max(Y1);
bR = 5:255;
bG = 5:255;
bB = 5:255;
bY = 5:255;
cR = histc(R1,bR);
cG = histc(G1,bG);
cB = histc(B1,bB);
cY = histc(Y1,bY);
[max_num, max_indexR] = max(cR);
[max_num2, max_indexG] = max(cG);
[max_num3, max_indexB] = max(cB);
[max_num4, max_indexY] = max(cY);
max_indexR=max_indexR+4;
max_indexG=max_indexG+4;
max_indexB=max_indexB+4;
max_indexY=max_indexY+4;

M = [max_indexR,max_indexG,max_indexB];
% M = [Rmax,Gmax,Bmax];
% YUVimag = rgb2ycbcr(f1);
% Y = YUVimag(:,:,1)
% figure,
% imshow(Y);
% MAX = max(M) 
%the num should be a 1x3 Integer mat limited in [0 255]
exchange_list={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
string='#';
for i=1:3
    temp_num=M(i);
    string(1+i*2-1)=exchange_list{(temp_num-mod(temp_num,16))/16+1};
    string(1+i*2)=exchange_list{mod(temp_num,16)+1};
end
  
HTML = string;
MAX = max(HTML);

axes(handles.axes16);
imhist(R),title('R-Histogram ') %显示小麦种子分离后绿色分辨率下的直方图
axes(handles.axes17);
imhist(G),title('G-component ')                           %显示小麦种子分离后G分量灰度图
axes(handles.axes18);
imhist(B),title('B-Histogram ')                          %显示小麦种子分离后蓝色分辨率下的直方图
% figure('NumberTitle','off','Name','RGB image and histogram after separation of wheat seeds'),            %小麦种子分离后的RGB图像和直方图
% subplot(4,2,1),imshow(f1),title('RGB image after separation')                                            %显示小麦种子分离后的RGB图像
% subplot(4,2,2),imhist(f1),title('Histogram of RGB image after separation')                               %显示小麦种子分离后RGB图像的直方图
% subplot(4,2,3),imshow(R),title('Grayscale image of R component after separation')                        %显示小麦种子分离后R分量灰度图
% subplot(4,2,4),imhist(R),title('Histogram at red resolution after separation')                           %显示小麦种子分离后绿色分辨率下的直方图
% subplot(4,2,5),imshow(G),title('G-component grayscale image after separation')                           %显示小麦种子分离后G分量灰度图
% subplot(4,2,6),imhist(G),title('Histogram at green resolution after separation')                         %显示小麦种子分离后绿色分辨率下的直方图
% subplot(4,2,7),imshow(B),title('G-component grayscale image after separation')                           %显示小麦种子分离后G分量灰度图
% subplot(4,2,8),imhist(B),title('Histogram at blue resolution after separation')                          %显示小麦种子分离后蓝色分辨率下的直方图
% hold on
% %set(gca,'YLim',[0 2000]);
% xlim=get(gca,'xlim');
% ylim=get(gca,'ylim');
% N=160;
% text(sum(xlim)/2-(1.42*N),sum(ylim)/2-(50*N),'Fourth wheat seed','horiz','center','FontSize',16,'FontWeight','Bold')
% 
% imwrite(frame,['D:\Seed discrimination\image_1\',num2str(clickcounts),'.jpg']);
% 
% R=double(R);G=double(G);B=double(B);
% hsv=rgb2hsv(f1);                                             %图像由rgb颜色空间转换到hsv颜色空间
% h=hsv(:,:,1);                                                %为色调h赋值                                   
% s=hsv(:,:,2);                                                %为饱和度s赋值
% v=hsv(:,:,3);                                                %为亮度v赋值                                                                       
%                                    
% figure('NumberTitle','off','Name','Hsv color space image and histogram after separation of wheat seeds'),  %小麦种子分离后的hsv颜色空间图像和直方图
% subplot(4,2,1),imshow(hsv),title('Hsv color space image after separation')                                 %显示小麦种子分离后的hsv颜色空间图像
% subplot(4,2,2),imhist(hsv),title('Hsv space image histogram after separation')                              %显示小麦种子图像分离后hsv颜色空间的直方图
% subplot(4,2,3),imshow(h),title('Grayscale image with hue h after separation')                               %显示小麦种子分离后色调h的灰度图
% subplot(4,2,4),imhist(h),title('Histogram based on hue h after separation')                                 %显示小麦种子分离后基于色调h的直方图
% subplot(4,2,5),imshow(s),title('Gray image of the saturation s after separation')                           %显示小麦种子分离后饱和度s的灰度图
% subplot(4,2,6),imhist(s),title('Histogram based on saturation s after separation')                          %显示小麦种子分离后基于饱和度s的直方图
% subplot(4,2,7),imshow(v),title('Gray image of brightness v after separation')                               %显示小麦种子分离后亮度v的灰度图
% subplot(4,2,8),imhist(v),title('Histogram based on brightness v after separation')                          %显示小麦种子分离后基于亮度v的直方图
% hold on
% %set(gca,'YLim',[0 2000]);
% xlim=get(gca,'xlim');
% ylim=get(gca,'ylim');
% N=200;
% text(sum(xlim)/2-(N/240),sum(ylim)/2-(40*N),'Fourth wheat seed','horiz','center','FontSize',16,'FontWeight','Bold')

[m,n]=size(f1);                             %求图像f1数据矩阵的大小赋值给[m,n],表示m×n维矩阵
mm=round(m/2);                              %对m/2取整赋值给mm
mn=round(n/2);                             
[p,q]=size(K);                             
pp=round(p/2);
qq=round(q/2);
f1=double(f1);                               %图像数据变为double型
% K=double(K);
colorsum=0.0;                              %给灰度值总和赋0值
Iavg1=mean2(f1);
Havg1=mean2(hsv);
f1=double(f1);
H=double(hsv); 
Ravg1=mean2(R);        %R分量的均值
Gavg1=mean2(G);         %G分量的均值
Bavg1=mean2(B);         %B分量的均值
Iavg=mean2(f1);          %求原图像一阶矩
R = double(R);
G = double(G);
B = double(B);
Rstd1=std(std(R));      %R分量的方差
Gstd1=std(std(G));      %G分量的方差
Bstd1=std(std(B));      %B分量的方差
Istd=std(std(f1));      %求原图像二阶矩

for i=1:mm                                 %循环求解灰度值总和   
    for j=1:mn
        colorsum=colorsum+(f1(i,j)-Iavg)^3;
    end
end

Iske=(colorsum/(mm*mn))^(1/3);                %求种子分离后RGB图像三阶矩 

% havg1=mean2(h)       %求h分量的灰度均值
% savg1=mean2(s)       %求s分量的灰度均值
% vavg1=mean2(v)       %求v分量的灰度均值
% hstd1=std(std(h))    %h分量的方差 
% sstd1=std(std(s))    %s分量的方差 
% vstd1=std(std(v))    %v分量的方差 
handles.Y4 = max_indexY;
handles.M4 = M;
handles.dd4 = f1;                                     %小麦种子分离后的RGB图像
guidata(hObject,handles);




% --- Executes on button press in pushbutton26.
function pushbutton26_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现     
z=handles.er;                                   %全部种子阈值分割后的二值图像                                                  
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
imshow(L_part5);title('Binary image after separation of wheat seeds');              %显示分离后的二值图像
handles.d5=L_part5;                                             %分离第五个小麦的二值图像
guidata(hObject,handles);









% --- Executes on button press in pushbutton30.
function pushbutton30_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
global status N5;
% clickcounts = clickcounts+1;
% set(hObject,'string',num2str(clickcounts));
axes(handles.axes5);
I=handles.f1;                                     %全部小麦种子原始图像
I1=handles.d5;                                    %分离第五个小麦的二值图像
for i=1:3
    I2(:,:,i)=I(:,:,i).*uint8(I1);
end
a = struct2cell(status);
x = a(N5);
x = cell2mat(x);
x1 = x(1);
y1 = x(2);
f1 = imcrop(I2,[x1,y1,125,125]);
imshow(f1);title('Seed remove background RGB image')
frame = f1;
h=ones(3,3)/9;                                   %线性滤波
K=imfilter(f1,h);
% figure,
% subplot(121);imshow(I2);title('原始图像')                         %显示原始图像
% subplot(122);imshow(K);title('线性滤波后的图像')                   %显示线性滤波后的图像
R=f1(:,:,1);                                      %原图像的R分量
G=f1(:,:,2);                                      %原图像的G分量
B=f1(:,:,3);                                      %原图像的B分量 

% Rc=R(:);
% Gc=G(:);
% Bc=B(:);
% MR=mode(Rc);
% MG=mode(Gc);
% MB=mode(Bc);
Y = R * 0.299 + G * 0.587 + B * 0.114;

R1=R(:);
G1=G(:);
B1=B(:);
Y1=Y(:);
Rmax=max(R1);
Gmax=max(G1);
Bmax=max(B1);
Ymax=max(Y1);
bR = 5:255;
bG = 5:255;
bB = 5:255;
bY = 5:255;
cR = histc(R1,bR);
cG = histc(G1,bG);
cB = histc(B1,bB);
cY = histc(Y1,bY);
[max_num, max_indexR] = max(cR);
[max_num2, max_indexG] = max(cG);
[max_num3, max_indexB] = max(cB);
[max_num4, max_indexY] = max(cY);
max_indexR=max_indexR+4;
max_indexG=max_indexG+4;
max_indexB=max_indexB+4;
max_indexY=max_indexY+4;
% Ymax = max(max(Y));
% Rmax = max(max(R));
% Gmax = max(max(G));
% Bmax = max(max(B));

M = [max_indexR,max_indexG,max_indexB];
% M = [MR,MG,MB];
YUVimag = rgb2ycbcr(f1);
% Y = YUVimag(:,:,1)
% figure,
% imshow(Y);
% MAX = max(M) 
%the num should be a 1x3 Integer mat limited in [0 255]
exchange_list={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
string='#';
for i=1:3
    temp_num=M(i);
    string(1+i*2-1)=exchange_list{(temp_num-mod(temp_num,16))/16+1};
    string(1+i*2)=exchange_list{mod(temp_num,16)+1};
end
  
HTML = string;
MAX = max(HTML);

axes(handles.axes19);
imhist(R),title('R-Histogram ') %显示小麦种子分离后绿色分辨率下的直方图
axes(handles.axes20);
imhist(G),title('G-component ')                           %显示小麦种子分离后G分量灰度图
axes(handles.axes21);
imhist(B),title('B-Histogram ')                          %显示小麦种子分离后蓝色分辨率下的直方图
% figure('NumberTitle','off','Name','RGB image and histogram after separation of wheat seeds'),            %小麦种子分离后的RGB图像和直方图
% subplot(4,2,1),imshow(f1),title('RGB image after separation')                                            %显示小麦种子分离后的RGB图像
% subplot(4,2,2),imhist(f1),title('Histogram of RGB image after separation')                               %显示小麦种子分离后RGB图像的直方图
% subplot(4,2,3),imshow(R),title('Grayscale image of R component after separation')                        %显示小麦种子分离后R分量灰度图
% subplot(4,2,4),imhist(R),title('Histogram at red resolution after separation')                           %显示小麦种子分离后绿色分辨率下的直方图
% subplot(4,2,5),imshow(G),title('G-component grayscale image after separation')                           %显示小麦种子分离后G分量灰度图
% subplot(4,2,6),imhist(G),title('Histogram at green resolution after separation')                         %显示小麦种子分离后绿色分辨率下的直方图
% subplot(4,2,7),imshow(B),title('G-component grayscale image after separation')                           %显示小麦种子分离后G分量灰度图
% subplot(4,2,8),imhist(B),title('Histogram at blue resolution after separation')                          %显示小麦种子分离后蓝色分辨率下的直方图
% hold on
% %set(gca,'YLim',[0 2000]);
% xlim=get(gca,'xlim');
% ylim=get(gca,'ylim');
% N=160;
% text(sum(xlim)/2-(1.45*N),sum(ylim)/2-(50*N),'Fifth wheat seed','horiz','center','FontSize',16,'FontWeight','Bold')
% 
% imwrite(frame,['D:\Seed discrimination\image_1\',num2str(clickcounts),'.jpg']);
% 
% R=double(R);G=double(G);B=double(B);
% hsv=rgb2hsv(f1);                                             %图像由rgb颜色空间转换到hsv颜色空间
% h=hsv(:,:,1);                                                %为色调h赋值                                   
% s=hsv(:,:,2);                                                %为饱和度s赋值
% v=hsv(:,:,3);                                                %为亮度v赋值                                                                       
%                                    
% figure('NumberTitle','off','Name','Hsv color space image and histogram after separation of wheat seeds'),  %小麦种子分离后的hsv颜色空间图像和直方图
% subplot(4,2,1),imshow(hsv),title('Hsv color space image after separation')                                 %显示小麦种子分离后的hsv颜色空间图像
% subplot(4,2,2),imhist(hsv),title('Hsv space image histogram after separation')                              %显示小麦种子图像分离后hsv颜色空间的直方图
% subplot(4,2,3),imshow(h),title('Grayscale image with hue h after separation')                               %显示小麦种子分离后色调h的灰度图
% subplot(4,2,4),imhist(h),title('Histogram based on hue h after separation')                                 %显示小麦种子分离后基于色调h的直方图
% subplot(4,2,5),imshow(s),title('Gray image of the saturation s after separation')                           %显示小麦种子分离后饱和度s的灰度图
% subplot(4,2,6),imhist(s),title('Histogram based on saturation s after separation')                          %显示小麦种子分离后基于饱和度s的直方图
% subplot(4,2,7),imshow(v),title('Gray image of brightness v after separation')                               %显示小麦种子分离后亮度v的灰度图
% subplot(4,2,8),imhist(v),title('Histogram based on brightness v after separation')                          %显示小麦种子分离后基于亮度v的直方图
% hold on
% %set(gca,'YLim',[0 2000]);
% xlim=get(gca,'xlim');
% ylim=get(gca,'ylim');
% N=200;
% text(sum(xlim)/2-(N/240),sum(ylim)/2-(40*N),'Fifth wheat seed','horiz','center','FontSize',16,'FontWeight','Bold')

[m,n]=size(f1);                             %求图像f1数据矩阵的大小赋值给[m,n],表示m×n维矩阵
mm=round(m/2);                              %对m/2取整赋值给mm
mn=round(n/2);                             
[p,q]=size(K);                             
pp=round(p/2);
qq=round(q/2);
f1=double(f1);                               %图像数据变为double型
% K=double(K);
colorsum=0.0;                              %给灰度值总和赋0值
Iavg1=mean2(f1);
Havg1=mean2(hsv);
f1=double(f1);
H=double(hsv); 
Ravg1=mean2(R);         %R分量的均值
Gavg1=mean2(G);         %G分量的均值
Bavg1=mean2(B);         %B分量的均值
Iavg=mean2(f1);          %求原图像一阶矩

R = double(R);
G = double(G);
B = double(B);

Rstd1=std(std(R));      %R分量的方差
Gstd1=std(std(G));      %G分量的方差
Bstd1=std(std(B));      %B分量的方差
Istd=std(std(f1));      %求原图像二阶矩

for i=1:mm                                 %循环求解灰度值总和   
    for j=1:mn
        colorsum=colorsum+(f1(i,j)-Iavg)^3;
    end
end

Iske=(colorsum/(mm*mn)).^(1/3);                %求种子分离后RGB图像三阶矩 

% havg1=mean2(h)       %求h分量的灰度均值
% savg1=mean2(s)       %求s分量的灰度均值
% vavg1=mean2(v)       %求v分量的灰度均值
% hstd1=std(std(h))    %h分量的方差 
% sstd1=std(std(s))    %s分量的方差 
% vstd1=std(std(v))    %v分量的方差 
handles.Y5 = max_indexY;
handles.M5 = M;
handles.dd5 = f1;                                     %小麦种子分离后的RGB图像
guidata(hObject,handles);




% --- Executes on button press in pushbutton32.
function pushbutton32_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现     
z=handles.er;                                   %全部种子阈值分割后的二值图像                                                  
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
imshow(L_part6);title('Binary image after separation of wheat seeds');              %显示分离后的二值图像
handles.d6=L_part6;                                             %分离第六个小麦的二值图像
guidata(hObject,handles);









% --- Executes on button press in pushbutton36.
function pushbutton36_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                      %取消警告
feature jit off                                  %加速代码实现
global status N6;
% clickcounts = clickcounts+1;
% set(hObject,'string',num2str(clickcounts));
axes(handles.axes6);
I=handles.f1;                                     %全部小麦种子原始图像
I1=handles.d6;                                    %分离第二个小麦的二值图像
for i=1:3
    I2(:,:,i)=I(:,:,i).*uint8(I1);
end
a = struct2cell(status);
x = a(N6);
x = cell2mat(x);
x1 = x(1);
y1 = x(2);
f1 = imcrop(I2,[x1,y1,125,125]);
imshow(f1);title('Seed remove background RGB image')
frame = f1;
h=ones(3,3)/9;                                   %线性滤波
K=imfilter(f1,h);
% figure,
% subplot(121);imshow(I2);title('原始图像')                         %显示原始图像
% subplot(122);imshow(K);title('线性滤波后的图像')                   %显示线性滤波后的图像
R=f1(:,:,1);                                      %原图像的R分量
G=f1(:,:,2);                                      %原图像的G分量
B=f1(:,:,3);                                      %原图像的B分量 

Y = R * 0.299 + G * 0.587 + B * 0.114;

% Ymax = max(max(Y));
% Rmax = max(max(R));
% Gmax = max(max(G));
% Bmax = max(max(B));
R1=R(:);
G1=G(:);
B1=B(:);
Y1=Y(:);
Rmax=max(R1);
Gmax=max(G1);
Bmax=max(B1);
Ymax=max(Y1);
bR = 5:255;
bG = 5:255;
bB = 5:255;
bY = 5:255;
cR = histc(R1,bR);
cG = histc(G1,bG);
cB = histc(B1,bB);
cY = histc(Y1,bY);
[max_num, max_indexR] = max(cR);
[max_num2, max_indexG] = max(cG);
[max_num3, max_indexB] = max(cB);
[max_num4, max_indexY] = max(cY);
max_indexR=max_indexR+4;
max_indexG=max_indexG+4;
max_indexB=max_indexB+4;
max_indexY=max_indexY+4;

M = [max_indexR,max_indexG,max_indexB];
% M = [Rmax,Gmax,Bmax];
YUVimag = rgb2ycbcr(f1);
% Y = YUVimag(:,:,1)
% figure,
% imshow(Y);
% MAX = max(M) 
%the num should be a 1x3 Integer mat limited in [0 255]
exchange_list={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
string='#';
for i=1:3
    temp_num=M(i);
    string(1+i*2-1)=exchange_list{(temp_num-mod(temp_num,16))/16+1};
    string(1+i*2)=exchange_list{mod(temp_num,16)+1};
end
  
HTML = string;
MAX = max(HTML);

axes(handles.axes22);
imhist(R),title('R-Histogram ') %显示小麦种子分离后绿色分辨率下的直方图
axes(handles.axes23);
imhist(G),title('G-component ')                           %显示小麦种子分离后G分量灰度图
axes(handles.axes24);
imhist(B),title('B-Histogram ')                          %显示小麦种子分离后蓝色分辨率下的直方图
% figure('NumberTitle','off','Name','RGB image and histogram after separation of wheat seeds'),            %小麦种子分离后的RGB图像和直方图
% subplot(4,2,1),imshow(f1),title('RGB image after separation')                                            %显示小麦种子分离后的RGB图像
% subplot(4,2,2),imhist(f1),title('Histogram of RGB image after separation')                               %显示小麦种子分离后RGB图像的直方图
% subplot(4,2,3),imshow(R),title('Grayscale image of R component after separation')                        %显示小麦种子分离后R分量灰度图
% subplot(4,2,4),imhist(R),title('Histogram at red resolution after separation')                           %显示小麦种子分离后绿色分辨率下的直方图
% subplot(4,2,5),imshow(G),title('G-component grayscale image after separation')                           %显示小麦种子分离后G分量灰度图
% subplot(4,2,6),imhist(G),title('Histogram at green resolution after separation')                         %显示小麦种子分离后绿色分辨率下的直方图
% subplot(4,2,7),imshow(B),title('G-component grayscale image after separation')                           %显示小麦种子分离后G分量灰度图
% subplot(4,2,8),imhist(B),title('Histogram at blue resolution after separation')                          %显示小麦种子分离后蓝色分辨率下的直方图
% 
% hold on
% %set(gca,'YLim',[0 2000]);
% xlim=get(gca,'xlim');
% ylim=get(gca,'ylim');
% N=160;
% text(sum(xlim)/2-(1.42*N),sum(ylim)/2-(50*N),'Sixth wheat seed','horiz','center','FontSize',16,'FontWeight','Bold')
% 
% imwrite(frame,['D:\Seed discrimination\image_1\',num2str(clickcounts),'.jpg']);
% 
% R=double(R);G=double(G);B=double(B);
% hsv=rgb2hsv(f1);                                             %图像由rgb颜色空间转换到hsv颜色空间
% h=hsv(:,:,1);                                                %为色调h赋值                                   
% s=hsv(:,:,2);                                                %为饱和度s赋值
% v=hsv(:,:,3);                                                %为亮度v赋值                                                                       
%                                    
% figure('NumberTitle','off','Name','Hsv color space image and histogram after separation of wheat seeds'),  %小麦种子分离后的hsv颜色空间图像和直方图
% subplot(4,2,1),imshow(hsv),title('Hsv color space image after separation')                                 %显示小麦种子分离后的hsv颜色空间图像
% subplot(4,2,2),imhist(hsv),title('Hsv space image histogram after separation')                              %显示小麦种子图像分离后hsv颜色空间的直方图
% subplot(4,2,3),imshow(h),title('Grayscale image with hue h after separation')                               %显示小麦种子分离后色调h的灰度图
% subplot(4,2,4),imhist(h),title('Histogram based on hue h after separation')                                 %显示小麦种子分离后基于色调h的直方图
% subplot(4,2,5),imshow(s),title('Gray image of the saturation s after separation')                           %显示小麦种子分离后饱和度s的灰度图
% subplot(4,2,6),imhist(s),title('Histogram based on saturation s after separation')                          %显示小麦种子分离后基于饱和度s的直方图
% subplot(4,2,7),imshow(v),title('Gray image of brightness v after separation')                               %显示小麦种子分离后亮度v的灰度图
% subplot(4,2,8),imhist(v),title('Histogram based on brightness v after separation')                          %显示小麦种子分离后基于亮度v的直方图
% hold on
% %set(gca,'YLim',[0 2000]);
% xlim=get(gca,'xlim');
% ylim=get(gca,'ylim');
% N=200;
% text(sum(xlim)/2-(N/240),sum(ylim)/2-(40*N),'Sixth wheat seed','horiz','center','FontSize',16,'FontWeight','Bold')

[m,n]=size(f1);                             %求图像f1数据矩阵的大小赋值给[m,n],表示m×n维矩阵
mm=round(m/2);                              %对m/2取整赋值给mm
mn=round(n/2);                             
[p,q]=size(K);                             
pp=round(p/2);
qq=round(q/2);
f1=double(f1);                               %图像数据变为double型
% K=double(K);
colorsum=0.0;                              %给灰度值总和赋0值
Iavg1=mean2(f1);
Havg1=mean2(hsv);
f1=double(f1);
H=double(hsv); 
Ravg1=mean2(R);         %R分量的均值
Gavg1=mean2(G);         %G分量的均值
Bavg1=mean2(B);         %B分量的均值
Iavg=mean2(f1);          %求原图像一阶矩

R = double(R);
G = double(G);
B = double(B);
Rstd1=std(std(R));      %R分量的方差
Gstd1=std(std(G));      %G分量的方差
Bstd1=std(std(B));      %B分量的方差
Istd=std(std(f1));      %求原图像二阶矩

for i=1:mm                                 %循环求解灰度值总和   
    for j=1:mn
        colorsum=colorsum+(f1(i,j)-Iavg)^3;
    end
end

Iske=(colorsum/(mm*mn)).^(1/3);                %求种子分离后RGB图像三阶矩 

% havg1=mean2(h)       %求h分量的灰度均值
% savg1=mean2(s)       %求s分量的灰度均值
% vavg1=mean2(v)       %求v分量的灰度均值
% hstd1=std(std(h))    %h分量的方差 
% sstd1=std(std(s))    %s分量的方差 
% vstd1=std(std(v))    %v分量的方差 
handles.Y6 = max_indexY;
handles.M6 = M;
handles.dd6 = f1;                                     %小麦种子分离后的RGB图像
guidata(hObject,handles);






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




function edit18_Callback(hObject, eventdata, handles)
% hObject    handle to edit18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit18 as text
%        str2double(get(hObject,'String')) returns contents of edit18 as a double


% --- Executes during object creation, after setting all properties.
function edit18_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit19_Callback(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit19 as text
%        str2double(get(hObject,'String')) returns contents of edit19 as a double


% --- Executes during object creation, after setting all properties.
function edit19_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function axes7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes7


% --- Executes during object creation, after setting all properties.
function pushbutton17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in pushbutton61.
function pushbutton61_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton61 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Y = handles.Y1;
M = handles.M1;
for i = 1:3
    temp = M(i);
    H(i) = temp/255;
    i = i + 1;
end
% disp(H);
set(handles.edit18,'backgroundcolor',H);
M = num2str(M);
a = strcat('RGB:', M);
set(handles.edit25,'string',a);
Y = num2str(Y);
b = strcat('Y:',Y);
set(handles.edit26,'string',b);
% a2 = handles.Y2;
% a3 = handles.Y3;
% a4 = handles.Y4;
% a5 = handles.Y5;
% a6 = handles.Y6;
% A = [a1 a2 a3 a4 a5 a6];
% S = sort(A,'descend');
% s = num2str(S);
% % disp(S);
% set(handles.edit18,'string',s);



function edit20_Callback(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit20 as text
%        str2double(get(hObject,'String')) returns contents of edit20 as a double


% --- Executes during object creation, after setting all properties.
function edit20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit21_Callback(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit21 as text
%        str2double(get(hObject,'String')) returns contents of edit21 as a double


% --- Executes during object creation, after setting all properties.
function edit21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit22_Callback(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit22 as text
%        str2double(get(hObject,'String')) returns contents of edit22 as a double


% --- Executes during object creation, after setting all properties.
function edit22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit23_Callback(hObject, eventdata, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit23 as text
%        str2double(get(hObject,'String')) returns contents of edit23 as a double


% --- Executes during object creation, after setting all properties.
function edit23_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit24_Callback(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit24 as text
%        str2double(get(hObject,'String')) returns contents of edit24 as a double


% --- Executes during object creation, after setting all properties.
function edit24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit25_Callback(hObject, eventdata, handles)
% hObject    handle to edit25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit25 as text
%        str2double(get(hObject,'String')) returns contents of edit25 as a double


% --- Executes during object creation, after setting all properties.
function edit25_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit26_Callback(hObject, eventdata, handles)
% hObject    handle to edit26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit26 as text
%        str2double(get(hObject,'String')) returns contents of edit26 as a double


% --- Executes during object creation, after setting all properties.
function edit26_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit27_Callback(hObject, eventdata, handles)
% hObject    handle to edit27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit27 as text
%        str2double(get(hObject,'String')) returns contents of edit27 as a double


% --- Executes during object creation, after setting all properties.
function edit27_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit28_Callback(hObject, eventdata, handles)
% hObject    handle to edit28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit28 as text
%        str2double(get(hObject,'String')) returns contents of edit28 as a double


% --- Executes during object creation, after setting all properties.
function edit28_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit29_Callback(hObject, eventdata, handles)
% hObject    handle to edit29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit29 as text
%        str2double(get(hObject,'String')) returns contents of edit29 as a double


% --- Executes during object creation, after setting all properties.
function edit29_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit30_Callback(hObject, eventdata, handles)
% hObject    handle to edit30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit30 as text
%        str2double(get(hObject,'String')) returns contents of edit30 as a double


% --- Executes during object creation, after setting all properties.
function edit30_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit31_Callback(hObject, eventdata, handles)
% hObject    handle to edit31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit31 as text
%        str2double(get(hObject,'String')) returns contents of edit31 as a double


% --- Executes during object creation, after setting all properties.
function edit31_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit32_Callback(hObject, eventdata, handles)
% hObject    handle to edit32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit32 as text
%        str2double(get(hObject,'String')) returns contents of edit32 as a double


% --- Executes during object creation, after setting all properties.
function edit32_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit33_Callback(hObject, eventdata, handles)
% hObject    handle to edit33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit33 as text
%        str2double(get(hObject,'String')) returns contents of edit33 as a double


% --- Executes during object creation, after setting all properties.
function edit33_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit34_Callback(hObject, eventdata, handles)
% hObject    handle to edit34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit34 as text
%        str2double(get(hObject,'String')) returns contents of edit34 as a double


% --- Executes during object creation, after setting all properties.
function edit34_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit35_Callback(hObject, eventdata, handles)
% hObject    handle to edit35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit35 as text
%        str2double(get(hObject,'String')) returns contents of edit35 as a double


% --- Executes during object creation, after setting all properties.
function edit35_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit36_Callback(hObject, eventdata, handles)
% hObject    handle to edit36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit36 as text
%        str2double(get(hObject,'String')) returns contents of edit36 as a double


% --- Executes during object creation, after setting all properties.
function edit36_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton62.
function pushbutton62_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton62 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Y = handles.Y2;
disp(Y)
M = handles.M2;
for i = 1:3
    temp = M(i);
    H(i) = temp/255;
    i = i + 1;
end

% disp(M);
set(handles.edit20,'backgroundcolor',H);
M = num2str(M);
a = strcat('RGB:', M);
set(handles.edit27,'string',a);
Y = num2str(Y);
b = strcat('Y:',Y);
set(handles.edit28,'string',b);


% --- Executes on button press in pushbutton63.
function pushbutton63_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton63 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Y = handles.Y3;
M = handles.M3;
for i = 1:3
    temp = M(i);
    H(i) = temp/255;
    i = i + 1;
end
% disp(M);
set(handles.edit21,'backgroundcolor',H);
M = num2str(M);
a = strcat('RGB:', M);
set(handles.edit29,'string',a);
Y = num2str(Y);
b = strcat('Y:',Y);
set(handles.edit30,'string',b);

% --- Executes on button press in pushbutton64.
function pushbutton64_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton64 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Y = handles.Y4;
M = handles.M4;
for i = 1:3
    temp = M(i);
    H(i) = temp/255;
    i = i + 1;
end
% disp(M);
set(handles.edit22,'backgroundcolor',H);
M = num2str(M);
a = strcat('RGB:', M);
set(handles.edit34,'string',a);
Y = num2str(Y);
b = strcat('Y:',Y);
set(handles.edit31,'string',b);

% --- Executes on button press in pushbutton65.
function pushbutton65_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton65 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Y = handles.Y5;
M = handles.M5;
for i = 1:3
    temp = M(i);
    H(i) = temp/255;
    i = i + 1;
end
% disp(M);
set(handles.edit23,'backgroundcolor',H);
M = num2str(M);
a = strcat('RGB:', M);
set(handles.edit35,'string',a);
Y = num2str(Y);
b = strcat('Y:',Y);
set(handles.edit32,'string',b);

% --- Executes on button press in pushbutton66.
function pushbutton66_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton66 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Y = handles.Y6;
M = handles.M6;
for i = 1:3
    temp = M(i);
    H(i) = temp/255;
    i = i + 1;
end
% disp(M);
set(handles.edit24,'backgroundcolor',H);

M = num2str(M);
a = strcat('RGB:', M);
set(handles.edit36,'string',a);
Y = num2str(Y);
b = strcat('Y:',Y);
set(handles.edit33,'string',b);
