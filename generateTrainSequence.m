function [pilot, trainSignal, txPilotStartIndices] =  generateTrainSequence( ...
        numTxAntennas, pilotLength, pilotToneFrequency, pilotAmplitude, guardIntervalLength, Ts)

    trainLength =  numTxAntennas*pilotLength ...
                + (numTxAntennas + 1) * guardIntervalLength;


    % Create time vector(Sample Frequency is Ts (Hz))
    t = [0:Ts:(pilotLength-1)*Ts].'; 

    % create the pilot signal
    signalPilot = pilotAmplitude * exp (t*j*2*pi*pilotToneFrequency);

    % this will be the zero vectore transmitted when another antennas is sending its training
    zeroPilot = zeros(size(signalPilot));

    % guard interval to place between different
    % fields within the packet. Mostly to eas visualization. 
    gaurdInterval = zeros(guardIntervalLength,1);


    trainSignal = zeros(trainLength ,numTxAntennas);

    pilotStencil = repmat([gaurdInterval; signalPilot],numTxAntennas,1);

    txPilotStartIndices = zeros(1,numTxAntennas);

    % orthogonal training symbols for each of the antennas
    for txAntenna = 1:numTxAntennas;
        % Indicator vector for which antenna we are crafting pilot for. 
        antennaIndicator = (txAntenna == 1:numTxAntennas);

        % Mask that will zero out the training pilot for all but slot assigned to active antenna
        orthogonalTrainMask = repmat(antennaIndicator, guardIntervalLength+pilotLength,1);
        orthogonalTrainMask = reshape(orthogonalTrainMask, numTxAntennas*(guardIntervalLength+pilotLength), 1);

        % Create the train signal
        trainSignal(:,txAntenna) = [orthogonalTrainMask .* pilotStencil; gaurdInterval];

        % txPilotStartIndices(txAntenna) = length(preamble) + (txAntenna-1) * pilotLength ...
        %                                         + txAntenna * guardIntervalLength + TXRX_DELAY;

        txPilotStartIndices(txAntenna) = (txAntenna-1) * pilotLength ...
                                                + txAntenna * guardIntervalLength;
    end

    pilot = signalPilot;


end