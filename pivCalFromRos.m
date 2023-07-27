% function pivCalFromRos()

rosinit

% number of samples
n = 100;

% optical tracker interface
pointerSub = rossubscriber('/NDI/PointerNew/local/measured_cp');

% init data place holder
pointerData = zeros(3*n,4);

% interaction - beep and wait for 20 seconds for operator to be ready
beep; pause(0.5); beep; pause(0.5); beep; pause(0.5);
pause(20);

% start collection
for i=1:n

    % receive a sample
    pointerDataCur = receive(pointerSub);

    % fill the translation
    t = pointerDataCur.Transform.Translation;
    pointerData((i-1)*3+1:(i-1)*3+3,4) = [t.X; t.Y; t.Z];

    % fill the rotation
    R = pointerDataCur.Transform.Rotation;
    pointerData((i-1)*3+1:(i-1)*3+3,1:3) = quat2rotm([R.W, R.X, R.Y, R.Z]);

    % samples interval
    pause(0.2);beep;

end

% finished
beep; pause(0.5); beep; pause(0.5); beep; pause(0.5); 

% calculation
matRI = [pointerData(:,1:3),zeros(3*n,3)];
for i=1:n
    matRI((i-1)*3+1:(i-1)*3+3,4:6) = -eye(3);
end
matT = -pointerData(:,4);
p = pinv(matRI) * matT;

rosshutdown

% end