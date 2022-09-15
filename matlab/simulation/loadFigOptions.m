
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [45 27]);
set(gcf, 'PaperPosition', [0 0 45  27]); 


xgraph = 0.15; %from left
%coordinates  for 4 subplots
four_y1 = 0.80;
four_y2 = 0.58;%s2nd graph
four_y3 = 0.38; %1st graph
four_y4 = 0.18; %1st graph
four_w = 0.8; %width
four_h = 0.2;
small_h = 0.13;
four_xgraph = 0.15;
%coordinates  for 3 subplots
three_y1 = 0.73;
three_y2 = 0.435;%s2nd graph
three_y3 = 0.14; %1st graph
three_w = 0.8; %width
three_h = 0.25;
three_xgraph =0.1;
%coordinates  for 2 subplots
two_xgraph = 0.1; %from left
two_y1 = 0.58;%s2nd graph
two_y2 = 0.15; %1st graph
two_w = 0.85; %width
two_h = 0.4;
%coordinates for 1 subplot
one_y1 = 0.2;
one_w = 0.8;
one_h = 0.7;
one_xgraph = 0.15;

%% Set grids on all axis.
set(0,'defaultAxesXGrid','on');
set(0,'defaultAxesYGrid','on');
set(0,'defaultAxesZGrid','on');


set(0,'defaultAxesXGrid','on');set(0,'defaultAxesYGrid','on');set(0,'defaultAxesZGrid','on');
% set some other default values
set(0, 'RecursionLimit', 50);set(0, 'DefaultFigurePaperType', 'A0');
set(0, 'Defaultlinelinewidth',3);set(0,'defaultaxeslinewidth',1)
set(0, 'defaultpatchlinewidth',1);set(0, 'DefaultFigureWindowStyle', 'normal');
set(0, 'DefaultAxesBox', 'on');set(0, 'DefaultTextFontSize', 40);
set(0, 'DefaultAxesFontSize', 40);set(0, 'DefaultUicontrolFontSize', 40);
set(0, 'Defaulttextinterpreter','latex')


