% analyse data from brute force csv:

%x,y,x2,y2,Fx,Fy,Fx2,Fy2
%binit(4), info(1), current(4), error(4)
%10,10,-10,-10,-1,0,-1,0

clear all
close all
data = csvread('datafile.csv',3,0);

binit = data(:,1:4);
info = data(:,5);
current = data(:,6:9);
error = data(:,10:end);
sumerror = sum(error,2);

% check for valid solutions
valididx = sumerror < 1e-15;

uniquecurrent = unique(current(valididx,:),'rows');

ratio13 = uniquecurrent(:,1)./uniquecurrent(:,3);
ratio24 = uniquecurrent(:,2)./uniquecurrent(:,4);

%% plot
plot(ratio24,ratio13,'xr')
axis equal
xlabel('ratio2/4')
ylabel('ratio1/3')

%% idx of max I1/I3:
sortratio13 = sort(unique(ratio13),1,'ascend');
uniqueidxmax13 = find(max(abs(ratio13))==abs(ratio13));
countunique = zeros(size(uniquecurrent,1),1);
for i=1:size(uniquecurrent,1)
    countunique(i) = sum((sum(bsxfun(@eq,current,uniquecurrent(i,:)),2) == 4));
    
    
end
sortratio13 = sort(ratio13,1,'ascend');
disp('Max ratio I1/I3')
currentmax13 = uniquecurrent(uniqueidxmax13,:)

% find which initial values lead to this soln:
for i = 1:size(currentmax13,1)
    %idxmax13 = find(sum(current == currentmax13(1,:),2) == 4);
    idxmax13 = find(sum(bsxfun(@eq,current,currentmax13(i,:)),2) == 4);
    
    disp('Initial current')
    binit(idxmax13,:)
    disp('Solution')
    current(idxmax13,:)
end

%% idx of max I1/I3:
disp('Next smallest ratio 13')
uniqueidxmax13 = find(abs(sortratio13(3))==abs(ratio13));
sortratio13 = sort(ratio13,1,'ascend');
disp('Ratio I1/I3')
sortratio13(3)
uniqueidxmax13 = 205;
currentmax13 = uniquecurrent(uniqueidxmax13,:)

% find which initial values lead to this soln:
for i = 1:size(currentmax13,1)
    %idxmax13 = find(sum(current == currentmax13(1,:),2) == 4);
    idxmax13 = find(sum(bsxfun(@eq,current,currentmax13(i,:)),2) == 4);
    
    disp('Initial current')
    binit(idxmax13,:)
    disp('Solution')
    %current(idxmax13,:)
end



