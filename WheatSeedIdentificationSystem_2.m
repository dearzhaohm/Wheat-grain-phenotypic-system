function varargout = WheatSeedIdentificationSystem_1(varargin)
% WHEATSEEDIDENTIFICATIONSYSTEM_2 MATLAB code for WheatSeedIdentificationSystem_2.fig
%      WHEATSEEDIDENTIFICATIONSYSTEM_2, by itself, creates a new WHEATSEEDIDENTIFICATIONSYSTEM_2 or raises the existing
%      singleton*.
%
%      H = WHEATSEEDIDENTIFICATIONSYSTEM_2 returns the handle to a new WHEATSEEDIDENTIFICATIONSYSTEM_2 or the handle to
%      the existing singleton*.
%
%      WHEATSEEDIDENTIFICATIONSYSTEM_2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in WHEATSEEDIDENTIFICATIONSYSTEM_2.M with the given input arguments.
%
%      WHEATSEEDIDENTIFICATIONSYSTEM_2('Property','Value',...) creates a new WHEATSEEDIDENTIFICATIONSYSTEM_2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before WheatSeedIdentificationSystem_2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to WheatSeedIdentificationSystem_2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help WheatSeedIdentificationSystem_2

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @WheatSeedIdentificationSystem_2_OpeningFcn, ...
                   'gui_OutputFcn',  @WheatSeedIdentificationSystem_2_OutputFcn, ...
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


% --- Executes just before WheatSeedIdentificationSystem_2 is made visible.
function WheatSeedIdentificationSystem_2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to WheatSeedIdentificationSystem_2 (see VARARGIN)

% Choose default command line output for WheatSeedIdentificationSystem_2
handles.output = hObject;
global clickcounts
clickcounts = 0;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes WheatSeedIdentificationSystem_2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = WheatSeedIdentificationSystem_2_OutputFcn(hObject, eventdata, handles) 
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
clc;
evalin('base','clear all *');                  %将工作空间的变量清除（evalin实现在工作空间中某个表达式字符串，并把结果返回）                 



% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现 

global vid
%定义一个全局变量
vid = videoinput('winvideo',1,'YUY2_640x480');
%winvideo函数可参照imaqhwinfo的用法
set(vid,'FramesPerTrigger',1);
%FramesPerTrigger用特定的帧数去触发捕获选定的视频源来显示视频
set(vid,'FrameGrabInterval',1);%FramesGrabInterval帧的抓取时间间隔
set(vid,'TriggerRepeat',Inf); 
%重复触发
set(vid,'ReturnedColorSpace','rgb');
%设置返回色彩rgb正常颜色（YUY2格式颜色发红，grayscale是灰度）
vidRes = get(vid,'VideoResolution');
%二维数组，获取摄像头的分辨率（即摄像头的宽和高）
obj = videoinput('winvideo');
nBands = get(obj,'NumberOfBands');
%获得摄像头数据的通道数（即图像的颜色层数）
axes(handles.axes1);
%让视频显示在指定的axes1坐标中
global hImage
%定义全局变量hImage
hImage = image(zeros(vidRes(2),vidRes(1),nBands));
%获得图像的句柄，hImage：视频预览窗口对应的句柄，也就是说在指定的句柄对象中预览视频，该格式可以空缺
preview(vid,hImage);%以hImage的尺寸格式显示摄像头数据
handles.f=vid;
guidata(hObject,handles);





% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

warning off                                    %取消警告
feature jit off                                %加速代码实现
vid = handles.f;
% close(findobj('gca','figure_csdn'));
% delete(vid);
closepreview(vid);
% close(findobj('gca','figure_csdn'));
%findobj('PropertyName','PropertyValue',...)
%返回figure的句柄
guidata(hObject,handles);

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vid clickcounts;
clickcounts = clickcounts+1;
axes(handles.axes1);
preview(vid);
frame = getsnapshot(vid);%获取摄像头每一帧的画面
figure;imshow(frame);

% [filename,pathname] = uiputfile({'*.bmp','BMP files';'*.jpg','JPG files'},'格式');
% %照片名称与储存路径
% if isequal(filename,0) || isequal(pathname,0)
% %文件名与路径是否被占用
% return;
% else
% fpath = fullfile(pathname,filename);
% end
%imwrite(frame,fpath);

imwrite(frame,['D:\Seed discrimination\vid get image\',num2str(clickcounts),'.jpg']);

set(vid,'ReturnedColorSpace','rgb');
% fig = gcf();
% clf(fig,'reset');
close;
guidata(hObject,handles);

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现
set(handles.edit14,'string','');
set(handles.edit15,'string','');
set(handles.edit16,'string','');
set(handles.edit17,'string','');
set(handles.edit18,'string','');
set(handles.edit19,'string','');
set(handles.edit20,'string','');
set(handles.edit21,'string','');
set(handles.edit22,'string','');
%从文件夹中选取图片并做处理
[filename,pathname]=uigetfile({'*.jpg';'*.*'},'请选择一张用于识别的照片');
filepath=[pathname,filename];                  %合并图像路径和名称
axes(handles.axes1);                           %在轴1下显示图像
data = imread(filepath);                       %读取图像
imshow(data)
r = data(:,:,1);
g = data(:,:,2);
b = data(:,:,3);
R = medfilt2(r);
G = medfilt2(g);
B = medfilt2(b);
data = cat(3,R,G,B);
% figure('NumberTitle','off','Name','Histogram of the original image of wheat seeds'), %小麦种子原始图像的直方图
% subplot(2,2,1),imhist(data),title('Histogram of raw image')                          %显示原图像的直方图
% subplot(2,2,2),imhist(R),title('Histogram of red')                                   %显示红色分辨率下的直方图
% subplot(2,2,3),imhist(G),title('Histogram of green')                                 %显示绿色分辨率下的直方图
% subplot(2,2,4),imhist(B),title('Histogram of blue')                                  %显示蓝色分辨率下的直方图;
% [data,rect] = imcrop(data);
% data = imresize(data,[227,227]); 
% rectangle('Position',rect,'LineWidth',2,'EdgeColor','r');
% data = data(240:1400,1160:3000,1:3);
%获取神经网络输入图像数据格式
%data1 = data(1:227,1:227,1:3);              %vgg16是 [224,224,3]

%最终交由VGG16识别的data1数组
handles.f1=data;                             %种子原始图像
guidata(hObject,handles);


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                       %取消警告
feature jit off                                   %加速代码实现
axes(handles.axes1);                              %在轴1下显示图像
I=handles.f1;                                      %原图像

grayimg = rgb2gray(I);                             %将原图像灰度化 

f2=imadjust(grayimg,[0.2 0.7],[0 1],4.8);          %灰度变换增强
BWimg =f2;
[width,height]=size(f2);
%二值化
T1=20;                                             %全局阈值分割
for i=1:width
   for j=1:height
      if(f2(i,j)<T1)
         BWimg(i,j)= 255;
      else
         BWimg(i,j)= 0;
      end
   end
end
s=medfilt2(BWimg,[3 3]);                                              %%3X3模板的中值滤波
s=im2double(s);
s=imfill(s,'holes');                                                   %空洞填充

f3=bwareaopen(s,200);                                                 %把图像中面积小于1200的区域去除
% figure,
% imshow(f3);
% figure('NumberTitle','off','Name','小麦种子的边缘提取'),                                                                                                  
imshow(f3),title('3X3 template median filtered image')                 %3X3模板的中值滤波图像

handles.f13=f3;                                                        %种子阈值分割后的二值图像
guidata(hObject,handles);

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                       %取消警告
feature jit off                                   %加速代码实现
axes(handles.axes1);                              %在轴1下显示图像
f=handles.f1;                                      %原图像
f1 = handles.f13;
se=strel('disk',5);
L = imclose(f1,se);                                                  %对小麦种子进行闭运算
f1 = imopen(L,se);                                                   %对小麦种子进行开运算
% imshow(f);
for i = 1:3
    h(:,:,i)=f(:,:,i).*uint8(f1);
end    
imshow(h),title('Seeds remove background image')                          %种子去背景图像
handles.f2 = h;
guidata(hObject,handles);



% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现
% global net_train
% [filename,pathname]=uigetfile({'*.mat';'*.*'},'请选择一种训练好的模型');
% filepath=[pathname,filename];                  %合并模型路径和名称
% net_train = importdata(filepath);
% assignin('base','net_train',net_train);        %为工作空间的变量指派值
[filename,pathname]=uigetfile({'*.jpg';'*.*'},'请选择一张用于识别的照片');
filepath=[pathname,filename];                  %合并图像路径和名称
axes(handles.axes1);                           %在轴1下显示图像
data = imread(filepath);                       %读取图像
imshow(data)
handles.s =  data;
guidata(hObject,handles);


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                       %取消警告
feature jit off                                   %加速代码实现
axes(handles.axes1);                              %在轴1下显示图像

popup_sel_index = get(handles.popupmenu1,'value');
switch popup_sel_index
    case 1
        data =  handles.s;
        imshow(data)
%         figure,
%         imshow(data);
%         data = imresize(data1,[227,227]); 
        %获取神经网络输入图像数据格式
        data1 = imresize(data,[227,227]);                 %vgg16是 [224,224,3]
        net_train = evalin('base','net_train');
        [label,conf] = classify(net_train,data1);
        imshow(data1);
        axes(handles.axes1);
        set(handles.edit6,'string',{char(label);max(conf)});
        % set(handles.edit2,'string',max(conf));
    case 2
        data =  handles.s;
        data1 = imresize(data,[227,227]); 
        %获取神经网络输入图像数据格式
%         data1 = data(1:227,1:227,1:3);                 %vgg16鏄痆224,224,3]
        net_train = evalin('base','net_train');
        [label,conf] = classify(net_train,data1);
        imshow(data1);
        axes(handles.axes1);
        set(handles.edit6,'string',{char(label);max(conf)});
        % set(handles.edit2,'string',max(conf));
    case 3
        data =  handles.s;
        data1 = imresize(data,[227,227]); 
        %获取神经网络输入图像数据格式
%         data1 = data(1:227,1:227,1:3);                 %vgg16鏄痆224,224,3]
        net_train = evalin('base','net_train');
        [label,conf] = classify(net_train,data1);
        imshow(data1);
        axes(handles.axes1);
        set(handles.edit6,'string',{char(label);max(conf)});
        % set(handles.edit2,'string',max(conf));
end
guidata(hObject,handles);
        
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.

function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现
clc;
close all;
evalin('base','clearvars *');    


function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

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



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

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






function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



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


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                        %取消警告
feature jit off                                    %加速代码实现
f10=handles.f13;
k=bwlabel(f10);                              %图像标记
n = regionprops(k==1,'MajorAxisLength');     %计算d第一个目标像素的长颈
n=struct2cell(n);
n=cell2mat(n);
% if n>=700
a=max(max(k))-1; 
% else
%     a=max(max(k));
% end
% a=max(max(k))-1;                           %计算目标个数
a1=bweuler(k,8);                             %计算欧拉数
set(handles.edit14,'string',a);
handles.f10=a;
guidata(hObject,handles);

% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                        %取消警告
feature jit off                                    %加速代码实现
f3=handles.f13;                                    %种子阈值分割后的二值图像
handles.f4=f3;
c=Subsystem_2;
set(c,'Visible','on');
a=handles.f4;
a1 = handles.f1;
save mat1 a;                                      %种子阈值分割后的二值图像
save mat2 a1;                                     %种子原始图像      


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



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现   
clc;
evalin('base','clearvars *');                  %将工作空间的变量清除（evalin实现在工作空间中某个表达式字符串，并把结果返回）  
net=alexnet;                                   %载入网络
layers = net.Layers(1:end-3);                  %去掉最后三层
new_layers = [
layers
fullyConnectedLayer(7,'WeightLearnRateFactor',20,'BiasLearnRateFactor',20)
softmaxLayer
% regressionLayer
classificationLayer
];


image = imageDatastore('train','IncludeSubfolders',true,'LabelSource','foldernames','ReadFcn',@IMAGERESIZE1);
[imageTrain,imageTest] = splitEachLabel(image,0.70,'randomized'); 
 %将数据集%70划为训练集，其余为测试集 
ops = trainingOptions('sgdm', ...
                      'InitialLearnRate',0.00001, ...
                      'ValidationData',imageTest, ...
                      'Plots','training-progress', ...
                      'MiniBatchSize',5, ...
                      'MaxEpochs',10,...
                      'ValidationPatience',Inf,...
                      'ExecutionEnvironment','gpu',...
                      'Verbose',false);
%'MaxEpochs' 即训练次数，我设的较小，根据需要调整
%开始训练
tic
net_train = trainNetwork(imageTrain,new_layers,ops);
assignin('base','net_train',net_train);        %为工作空间的变量指派值
toc

% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现  
clc;
evalin('base','clearvars *');                  %将工作空间的变量清除（evalin实现在工作空间中某个表达式字符串，并把结果返回）  
net = vgg16;                                   %载入网络
layers = net.Layers(1:end-3);                  %去掉最后三层
new_layers = [
layers
fullyConnectedLayer(7,'WeightLearnRateFactor',20,'BiasLearnRateFactor',20)
softmaxLayer
classificationLayer
];


image = imageDatastore('train','IncludeSubfolders',true,'LabelSource','foldernames','ReadFcn',@IMAGERESIZE);
[imageTrain,imageTest] = splitEachLabel(image,0.70,'randomized'); 
 %将数据集%70划为训练集，其余为测试集 
ops = trainingOptions('sgdm', ...
                      'InitialLearnRate',0.01, ...
                      'ValidationData',imageTest, ...
                      'Plots','training-progress', ...
                      'MiniBatchSize',10, ...
                      'MaxEpochs',10,...
                      'ValidationPatience',Inf,...
                      'ExecutionEnvironment','gpu',...
                      'Verbose',false);
%'MaxEpochs' 即训练次数，我设的较小，根据需要调整
%开始训练
tic
net_train = trainNetwork(imageTrain,new_layers,ops);
assignin('base','net_train',net_train,'doTraining',doTraining);        %为工作空间的变量指派值
toc

% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现
clc;
evalin('base','clearvars *');                  %将工作空间的变量清除（evalin实现在工作空间中某个表达式字符串，并把结果返回）  
net = vgg19;                                   %载入网络
layers = net.Layers(1:end-3);                  %去掉最后三层
new_layers = [
layers
fullyConnectedLayer(7,'WeightLearnRateFactor',20,'BiasLearnRateFactor',20)
softmaxLayer
regressionLayer
% classificationLayer
];


image = imageDatastore('train','IncludeSubfolders',true,'LabelSource','foldernames','ReadFcn',@IMAGERESIZE);
[imageTrain,imageTest] = splitEachLabel(image,0.70,'randomized'); 
 %将数据集%70划为训练集，其余为测试集 
ops = trainingOptions('sgdm', ...
                      'InitialLearnRate',0.0001, ...
                      'ValidationData',imageTest, ...
                      'Plots','training-progress', ...
                      'MiniBatchSize',10, ...
                      'MaxEpochs',10,...
                      'ValidationPatience',Inf,...
                      'ExecutionEnvironment','gpu',...
                      'Verbose',false);
%'MaxEpochs' 即训练次数，我设的较小，根据需要调整
%开始训练
tic
net_train = trainNetwork(imageTrain,new_layers,ops);
assignin('base','net_train',net_train,'doTraining',doTraining);        %为工作空间的变量指派值
toc

% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现 
clc;
evalin('base','clearvars *');                  %将工作空间的变量清除（evalin实现在工作空间中某个表达式字符串，并把结果返回）  
net=resnet50;                                  %w载入网络
layers = net.Layers(1:end-3);                  %去掉最后三层
new_layers = [
layers
fullyConnectedLayer(7,'Name','fc3','WeightLearnRateFactor',20,'BiasLearnRateFactor',20)
softmaxLayer('Name','fc3_softmax')
classificationLayer('Name','ClassificationLayer_fc3')
];
lgraph = layerGraph(new_layers);
figure;plot(lgraph)

%修改层连接
lgraph = removeLayers(lgraph,'res2a_branch1');
lgraph = removeLayers(lgraph,'bn2a_branch1');
lgraph = removeLayers(lgraph,'res3a_branch1');
lgraph = removeLayers(lgraph,'bn3a_branch1');
lgraph = removeLayers(lgraph,'res4a_branch1');
lgraph = removeLayers(lgraph,'bn4a_branch1');
lgraph = removeLayers(lgraph,'res5a_branch1');
lgraph = removeLayers(lgraph,'bn5a_branch1');
figure;plot(lgraph)

layers_1 = lgraph.Layers;
lgraph_1 = layerGraph(layers_1);
figure;plot(lgraph_1)
%添加层
res2a_branch1 = convolution2dLayer(1,256,'Name','res2a_branch1','Stride',1);
bn2a_branch1 = batchNormalizationLayer('Name','bn2a_branch1');
res3a_branch1 = convolution2dLayer(1,512,'Name','res3a_branch1','Stride',2);
bn3a_branch1 = batchNormalizationLayer('Name','bn3a_branch1');
res4a_branch1 = convolution2dLayer(1,1024,'Name','res4a_branch1','Stride',2);
bn4a_branch1 = batchNormalizationLayer('Name','bn4a_branch1');
res5a_branch1 = convolution2dLayer(1,2048,'Name','res5a_branch1','Stride',2);
bn5a_branch1 = batchNormalizationLayer('Name','bn5a_branch1');
lgraph_1 = addLayers(lgraph_1,res2a_branch1);
lgraph_1 = addLayers(lgraph_1,bn2a_branch1);
lgraph_1 = addLayers(lgraph_1,res3a_branch1);
lgraph_1 = addLayers(lgraph_1,bn3a_branch1);
lgraph_1 = addLayers(lgraph_1,res4a_branch1);
lgraph_1 = addLayers(lgraph_1,bn4a_branch1);
lgraph_1 = addLayers(lgraph_1,res5a_branch1);
lgraph_1 = addLayers(lgraph_1,bn5a_branch1);
figure;plot(lgraph_1)

%修改连接
lgraph_1 = connectLayers(lgraph_1,'max_pooling2d_1','res2a_branch1');
lgraph_1 = connectLayers(lgraph_1,'res2a_branch1','bn2a_branch1');
lgraph_1 = connectLayers(lgraph_1,'bn2a_branch1','add_1/in2');
% ------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_4_relu','add_2/in2');
% -------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_7_relu','add_3/in2');
% ----------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_10_relu','res3a_branch1');
lgraph_1 = connectLayers(lgraph_1,'res3a_branch1','bn3a_branch1');
lgraph_1 = connectLayers(lgraph_1,'bn3a_branch1','add_4/in2');
% ------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_13_relu','add_5/in2');
% -------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_16_relu','add_6/in2');
% -------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_19_relu','add_7/in2');
% ----------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_22_relu','res4a_branch1');
lgraph_1 = connectLayers(lgraph_1,'res4a_branch1','bn4a_branch1');
lgraph_1 = connectLayers(lgraph_1,'bn4a_branch1','add_8/in2');
% ------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_25_relu','add_9/in2');
% -------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_28_relu','add_10/in2');
% -------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_31_relu','add_11/in2');
% -------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_34_relu','add_12/in2');
% -------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_37_relu','add_13/in2');
% -------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_40_relu','res5a_branch1');
lgraph_1 = connectLayers(lgraph_1,'res5a_branch1','bn5a_branch1');
lgraph_1 = connectLayers(lgraph_1,'bn5a_branch1','add_14/in2');
% ------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_43_relu','add_15/in2');
% -------------------------------------------------------------------------------------
lgraph_1 = connectLayers(lgraph_1,'activation_46_relu','add_16/in2');
figure;plot(lgraph_1)

image = imageDatastore('train','IncludeSubfolders',true,'LabelSource','foldernames','ReadFcn',@IMAGERESIZE);
[imageTrain,imageTest] = splitEachLabel(image,0.70,'randomized'); 
 %将数据集%70划为训练集，其余为测试集 
ops = trainingOptions('adam', ...
                      'InitialLearnRate',0.0001, ...
                      'ValidationData',imageTest, ...
                      'Plots','training-progress', ...
                      'MiniBatchSize',20, ...
                      'MaxEpochs',10,...
                      'ValidationPatience',Inf,...
                      'ExecutionEnvironment','gpu',...
                      'Verbose',false);
%'MaxEpochs' 即训练次数，我设的较小，根据需要调整
%开始训练
tic
doTraining = true;
net_train = trainNetwork(imageTrain,lgraph_1,ops);
assignin('base','net_train',net_train);        %为工作空间的变量指派值
assignin('base','doTraining',doTraining);      %为工作空间的变量指派值
toc


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


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现
global net_train
[filename,pathname]=uigetfile({'*.mat';'*.*'},'请选择一种训练好的模型');
filepath=[pathname,filename];                  %合并模型路径和名称
net_train = importdata(filepath);
assignin('base','net_train1',net_train);        %为工作空间的变量指派值
guidata(hObject,handles);


% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                        %取消警告
feature jit off                                    %加速代码实现
A=handles.f13;
%先闭运算 再开运算
se=strel('disk',5);
% L = imclose(f3,se);                                                  %对小麦种子进行开运算
A = imopen(A,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(A,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
% disp(num);
n = regionprops(L==1,'MajorAxisLength');       %计算d第一个目标像素的长颈
n=struct2cell(n);
n=cell2mat(n);
s = 0;
P = [];
for i=2:num
    f = (L == 1);
    f1= (L == i);
    total=bwarea(f1);                               %小麦的像素面积
    total2=bwarea(f);                               %硬币的像素面积
    c = 551.266;                                     %硬币的实际面积
    p = (c*total)/total2;                            %小麦的实际面积
    disp(c)
    i = i + 1;
    s = s+ p;
end
    s = s/(num - 1);
totalp=num2str(s);
set(handles.edit15,'string',totalp);
handles.Amean = s;
guidata(hObject,handles);


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


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                        %取消警告
feature jit off                                    %加速代码实现
A1=handles.f13;
%先闭运算 再开运算
se=strel('disk',5);
% L = imclose(f3,se);                                                  %对小麦种子进行开运算
A1 = imopen(A1,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(A1,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
% disp(num);
n = regionprops(L==1,'MajorAxisLength');       %计算d第一个目标像素的长颈
n=struct2cell(n);
n=cell2mat(n);
S = [];
% if n>=700
for i=2:num
    f = (L == 1);
    f1= (L == i);
    total=bwarea(f1);                               %小麦的像素面积
    total2=bwarea(f);                               %硬币的像素面积
    c = 551.266;                                     %硬币的实际面积
    p = (c*total)/total2;                            %小麦的实际面积
    S(i-1) =  p;
    i = i + 1;        
end
    s = std(S);
% end
totalp=num2str(s);
m1 = max(max(S));
n1 = min(min(S));
set(handles.edit16,'String',totalp);
handles.Astd = s;
handles.m1 = m1;
handles.n1 = n1;
handles.S1 = S;
guidata(hObject,handles);


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


% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                        %取消警告
feature jit off                                    %加速代码实现
format long;
A=handles.f13;
%先闭运算 再开运算
se=strel('disk',5);
% L = imclose(f3,se);                                                  %对小麦种子进行开运算
A = imopen(A,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(A,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
%disp(num);
n = regionprops(L==1,'MajorAxisLength');       %计算d第一个目标像素的长颈
n=struct2cell(n);
n=cell2mat(n);
s = 0;

for i=2:num
    f = logical(L == 1);
    f1= logical(L == i);
    a = regionprops(f,'MajorAxisLength');         %计算硬币像素的直径
    a1=regionprops(f1,'MajorAxisLength');         %计算小麦像素的长颈
    a = struct2cell(a);
    a1 = struct2cell(a1);
    a=cell2mat(a);
    a1=cell2mat(a1);
    c = 25;                                        %硬币的实际直径
    p = (c*a1)/a;                                  %小麦的实际长颈
    i = i + 1;
    s = s+ p;
end
s = s/(num - 1);
totalp=num2str(s);
set(handles.edit17,'string',totalp);
handles.Lmean = s;
guidata(hObject,handles);


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


% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                        %取消警告
feature jit off                                    %加速代码实现
format long;
A1=handles.f13;
%先闭运算 再开运算
se=strel('disk',5);
% L = imclose(f3,se);                                                  %对小麦种子进行开运算
A1 = imopen(A1,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(A1,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
%disp(num);
n = regionprops(L==1,'MajorAxisLength');       %计算d第一个目标像素的长颈
n=struct2cell(n);
n=cell2mat(n);
S = [];

for i=2:num
    f = logical(L == 1);
    f1= logical(L == i);
    a = regionprops(f,'MajorAxisLength');         %计算硬币像素的直径
    a1=regionprops(f1,'MajorAxisLength');         %计算小麦像素的长颈
    a = struct2cell(a);
    a1 = struct2cell(a1);
    a=cell2mat(a);
    a1=cell2mat(a1);
    c = 25;                                        %硬币的实际直径
    p = (c*a1)/a;                                  %小麦的实际长颈
    S(i-1) = p;
    i = i + 1;
end
s = std(S);
totalp=num2str(s);
m2 = max(max(S));
n2 = min(min(S));
set(handles.edit18,'string',totalp);
handles.Lstd = s;
handles.m2 = m2;
handles.n2 = n2;
handles.S2 = S;
guidata(hObject,handles);


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


% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                        %取消警告
feature jit off                                    %加速代码实现
format long;
A=handles.f13;
%先闭运算 再开运算
se=strel('disk',5);
% L = imclose(f3,se);                                                  %对小麦种子进行开运算
A = imopen(A,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(A,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
%disp(num);
n = regionprops(L==1,'MajorAxisLength');       %计算d第一个目标像素的长颈
n=struct2cell(n);
n=cell2mat(n);
s = 0;
for i=2:num
    f = logical(L == 1);
    f1= logical(L == i);
    a = regionprops(f,'MinorAxisLength');         %计算硬币像素的直径
    a1=regionprops(f1,'MinorAxisLength');         %计算小麦像素的短颈
    a = struct2cell(a);
    a1 = struct2cell(a1);
    a=cell2mat(a);
    a1=cell2mat(a1);
    c = 25;                                        %硬币的实际直径
    p = (c*a1)/a;                                  %小麦的实际短颈
    i = i + 1;
    s = s+ p;
end
s = s/(num - 1);
totalp=num2str(s);
set(handles.edit19,'string',totalp);
handles.Smean = s;
guidata(hObject,handles);


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


% --- Executes on button press in pushbutton23.
function pushbutton23_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                        %取消警告
feature jit off                                    %加速代码实现
format long;
A1=handles.f13;
%先闭运算 再开运算
se=strel('disk',5);
% L = imclose(f3,se);                                                  %对小麦种子进行开运算
A1 = imopen(A1,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(A1,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
%disp(num);
n = regionprops(L==1,'MajorAxisLength');       %计算d第一个目标像素的长颈
n=struct2cell(n);
n=cell2mat(n);
S = [];
for i=2:num
    f = logical(L == 1);
    f1= logical(L == i);
    a = regionprops(f,'MinorAxisLength');         %计算硬币像素的直径
    a1=regionprops(f1,'MinorAxisLength');         %计算小麦像素的短颈
    a = struct2cell(a);
    a1 = struct2cell(a1);
    a=cell2mat(a);
    a1=cell2mat(a1);
    c = 25;                                        %硬币的实际直径
    p = (c*a1)/a;                                  %小麦的实际短颈
    S(i-1) = p;
    i = i + 1;
end
s = std(S);
totalp=num2str(s);
m3 = max(max(S));
n3 = min(min(S));
set(handles.edit20,'string',totalp);
handles.Sstd = s;
handles.m3 = m3;
handles.n3 = n3;
handles.S3 = S;
guidata(hObject,handles);


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


% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                        %取消警告
feature jit off                                    %加速代码实现
format long;
A=handles.f13;
%先闭运算 再开运算
se=strel('disk',5);
% L = imclose(f3,se);                                                  %对小麦种子进行开运算
A = imopen(A,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(A,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
%disp(num);
n = regionprops(L==1,'MajorAxisLength');       %计算d第一个目标像素的长颈
n=struct2cell(n);
n=cell2mat(n);
s = 0;
for i=2:num
    f = logical(L == 1);
    f1= logical(L == i);
    a = regionprops(f,'MajorAxisLength');         %计算硬币像素的直径
    a1=regionprops(f1,'MajorAxisLength');         %计算小麦像素的长颈
    a = struct2cell(a);
    a1 = struct2cell(a1);
    a=cell2mat(a);
    a1=cell2mat(a1);
    c = 25;                                        %硬币的实际直径
    p1 = (c*a1)/a;                                  %小麦的实际长颈

    a_1=regionprops(f1,'MinorAxisLength');         %计算小麦像素的短颈
    a_1 = struct2cell(a_1);
    a_1=cell2mat(a_1);
    p2 = (c*a_1)/a;                                  %小麦的实际短颈
    p = p1/p2;
    i = i + 1;
    s = s+ p;
end
s = s/(num - 1);
totalp=num2str(s);
set(handles.edit21,'string',totalp);
handles.Rmean = s;
guidata(hObject,handles);



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


% --- Executes on button press in pushbutton25.
function pushbutton25_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                        %取消警告
feature jit off                                    %加速代码实现
format long;
A=handles.f13;
%先闭运算 再开运算
se=strel('disk',5);
% L = imclose(f3,se);                                                  %对小麦种子进行开运算
A = imopen(A,se);                                                   %对小麦种子进行闭运算
[L,num]=bwlabel(A,4);                                               %对形态学操作后的图像进行标记
%bwlabel 寻找连通区域， 4连通是指，如果像素的位置在其他像素相邻的上、下、左或右，则认为他们是连接着的
%num 表示连通区域的个数
%disp(num);
n = regionprops(L==1,'MajorAxisLength');       %计算d第一个目标像素的长颈
n=struct2cell(n);
n=cell2mat(n);
S = [];
for i=2:num
    f = logical(L == 1);
    f1= logical(L == i);
    a = regionprops(f,'MajorAxisLength');         %计算硬币像素的直径
    a1=regionprops(f1,'MajorAxisLength');         %计算小麦像素的长颈
    a = struct2cell(a);
    a1 = struct2cell(a1);
    a=cell2mat(a);
    a1=cell2mat(a1);
    c = 25;                                        %硬币的实际直径
    p1 = (c*a1)/a;                                  %小麦的实际长颈

    a_1=regionprops(f1,'MinorAxisLength');         %计算小麦像素的短颈
    a_1 = struct2cell(a_1);
    a_1=cell2mat(a_1);
    p2 = (c*a_1)/a;                                  %小麦的实际短颈
    p = p1/p2;
    S(i-1) = p;
    i = i + 1;
end
s = std(S);

m4 = max(max(S));
n4 = min(min(S));
totalp=num2str(s);
set(handles.edit22,'string',totalp);
handles.Rstd = s;
handles.m4 = m4;
handles.n4 = n4;
handles.S4 = S;
guidata(hObject,handles);


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


% --- Executes on button press in pushbutton26.
function pushbutton26_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                    %取消警告
feature jit off                                %加速代码实现
global clickcounts ;
clickcounts = clickcounts+1;
a1 = handles.f10;
a2 = handles.Amean;
a3 = handles.Astd ;
a4 = handles.m1 ;
a5 = handles.n1 ;
a6 = handles.Lmean ;
a7 = handles.Lstd;
a8 = handles.m2;
a9 = handles.n2;
a10 = handles.Smean;
a11 = handles.Sstd ;
a12 = handles.m3;
a13 = handles.n3;
a14 = handles.Rmean;
a15 = handles.Rstd ;
a16 = handles.m4 ;
a17 = handles.n4;
A = handles.S1; %各个小麦种子的面积
L = handles.S2; %各个小麦种子的长颈                         
S = handles.S3; %各个小麦种子的短颈 
R = handles.S4; %各个小麦种子的长颈和短颈比
T1 = [A,a1,a2,a3,a4,a5]; 
T2 = [L,a1,a6,a7,a8,a9];
T3 = [S,a1,a10,a11,a12,a13];
T4 = [R,a1,a14,a15,a16,a17];
% T = [T1;T2;T3;T4];
T1=num2cell(T1);
T2=num2cell(T2);
T3=num2cell(T3);
T4=num2cell(T4);
%行名称
BD={'Area';'Longneck';'Shortneck';'Rate'}; 

% b=strcat('a',num2str(1))
%列名称
count =a1;
various = {};
name={'Count','Mean','Std','Max','Min'};
for i = 1:count+5
    if i <= count
%         if i == 1
%             various(i) = {'shuxing'};
%         else
        various(i) = {num2str(i)};
%         end
    else
        various(i) = name(i-count);
    end
end


field0 = 'shuxing';value0 = various;
field1 = 'Area';value1 = T1;
field2 = 'Longneck';value2 = T2;
field3 = 'Shortneck';value3 = T3;
field4 = 'Rate';value4 = T4;
s = struct(field0,value0,field1,value1,field2,value2,field3,value3,field4,value4);

% various
%生成表格，按列生成
% for i = 1:count+5
% various(1) = T(:,1);
% result_table=table(various(1),'VNames',BD);
% i = i + 1;
% end
% result_table=table(BD,t(:,1),'RowNames',BD);
S = struct2cell(s);
num2str(clickcounts);
%保存表格
% writecell(S, ['D:\种子照片',num2str(clickcounts),'.xlsx']);  %Excel格式
writecell(S, ['D:\种子照片',num2str(clickcounts),'.csv']);   %csv格式


% --- Executes on button press in pushbutton27.
function pushbutton27_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warning off                                        %取消警告
feature jit off                                    %加速代码实现
f3=handles.f13;                                    %种子阈值分割后的二值图像

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
global status 
status=regionprops(L,'BoundingBox');
centroid=regionprops(L,'Centroid');
%返回值STATS是一个长度为max(L(:))的结构数组，结构数组的相应域定义了每一个区域相应属性下的度量
for i=1:num
    rectangle('position',status(i).BoundingBox,'edgecolor','r');
    %参数说明：position绘制的为二维图像（他是通过对角的两点确定矩形框）
    %edgecolor 指边缘图像，r表示变换为红色。
    %facecolor 指内部填充颜色。
    text(centroid(i,1).Centroid(1,1)-10,centroid(i,1).Centroid(1,2)-20, num2str(i),'Color', 'r')
    %这个是为绘制出来的矩形框图标记数字
end


handles.f4=f3;
c=Subsystem_xiugai1;
set(c,'Visible','on');
a=handles.f4;
a1 = handles.f1;

save mat1 a;                                      %种子阈值分割后的二值图像
save mat2 a1;                                     %种子原始图像
