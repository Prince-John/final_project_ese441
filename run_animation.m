
%%
close all

I = imread('sky.jpg');
hold on

for k = 1:length(simout.Time)
    
    ang = simout.Data(k);
    
    rotate = [cos(ang) -sin(ang);
        sin(ang) cos(ang)];
    
    clf
    hold on
    
    h = image([-6 6],[-6 6],I);
        
    wing = [[-5; .5] [0;-.5] [5; .5]];
    tail1 = [[0; 0] [0;0] [0; 2.3]];
    tail2 = [[-1.6; .6] [0;.2] [1.6; .6]];
    window = [[-.7; .3] [0;.3] [.7; .3]];
    engine = [[-1.8; -.3] [0;0] [1.8; -.3]];
    
    RWing = zeros(size(wing));
    RTail1 = zeros(size(tail1));
    RTail2 = zeros(size(tail2));
    RWindow = zeros(size(window));
    REngine = zeros(size(window));
    
    for i=1:size(wing,2)        
        RWing(:,i) = rotate*wing(:,i);        
        RTail1(:,i) = rotate*tail1(:,i);        
        RTail2(:,i) = rotate*tail2(:,i);        
        RWindow(:,i) = rotate*window(:,i);        
        REngine(:,i) = rotate*engine(:,i);        
    end
    
    
    % plot wings
    plot(RWing(1,:),RWing(2,:),'Color','b', 'Linewidth',5)
    
    % plot wings
    plot(REngine(1,:),REngine(2,:),'.k','MarkerSize',70)
    
    % plot tail1
    plot(RTail1(1,:),RTail1(2,:),'Color','b', 'Linewidth',4)
    
    % plot tail2
    plot(RTail2(1,:),RTail2(2,:),'Color','b', 'Linewidth',4)
    
    % plot fuselage
    plot(0,0,'r.','MarkerSize',140);
    
    % plot window
    plot(RWindow(1,:),RWindow(2,:),'w','Linewidth',8);
    
    title(strcat('Time= ',num2str((k-1)*0.05)),'FontSize',20);
    
    axis equal
    axis([-6,6,-6 6])    
    xticks([]);
    yticks([]);
        
    drawnow
    
end