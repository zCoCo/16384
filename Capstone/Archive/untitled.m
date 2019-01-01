
gravityVec = -[fbk.accelX(1) fbk.accelY(1) fbk.accelZ(1)];

% dealing with non identity base frames
baseFrame = kin.getBaseFrame();
gravityVec = baseFrame(1:3,1:3)' * gravityVec;