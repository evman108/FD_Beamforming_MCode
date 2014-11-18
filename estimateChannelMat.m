function H = estimateChannelMat(numTxAntennas, numRxAntennas, rx_IQ, pilotIndices, pilotReference)
    
	pilotLength = length(pilotReference);
    pilots = zeros(pilotLength,0);
    for txAntenna = 1:numTxAntennas
        pilots = [pilots, rx_IQ(pilotIndices(txAntenna)+1 : ...
                                  pilotIndices(txAntenna)+pilotLength,:)];
    end

    H = reshape(mean(pilots ./ repmat(pilotReference,1,numTxAntennas*numRxAntennas)),numRxAntennas,numTxAntennas);