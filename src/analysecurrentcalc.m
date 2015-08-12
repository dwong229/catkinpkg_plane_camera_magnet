% analyse data from brute force csv:

%x,y,x2,y2,Fx,Fy,Fx2,Fy2
%binit(4), info(1), current(4), error(4)
%10,10,-10,-10,-1,0,-1,0

data = csvread('datafile.csv',3,0);


binit = data(:,1:4);
info = data(:,5);
current = data(:,6:9);
error = data(:,10:end);
sumerror = sum(error,2);


valididx = info == 2 & sumerror < 1e-15;


unique(current(valididx,:),'rows')

% Valid solns if info == 2 and error< 1e-10
%for i = 1:length(binit)
%    if 
%        % valid soln
%        binit_valid
%    
%end