
centerLocation = [39.745 -8.8065];

% Define center location site (cells 1-3)
centerSite = txsite('Name','MathWorks Glasgow', ...
    'Latitude',centerLocation(1),...
    'Longitude',centerLocation(2));

% Initialize arrays for distance and angle from center location to each cell site, where
% each site has 3 cells
numCellSites = 6;
siteDistances = zeros(1,numCellSites);
siteAngles = zeros(1,numCellSites);

% Define distance and angle for inner ring of 6 sites (cells 4-21)
isd = 510/cosd(30); % Inter-site distance   % 500/cosd(30)
siteDistances(2:7) = isd;
siteAngles(2:7) = 30:60:360;

% Define distance and angle for middle ring of 6 sites (cells 22-39)
siteDistances(8:13) = 2*isd*cosd(30);
siteAngles(8:13) = 0:60:300;

% Define distance and angle for outer ring of 6 sites (cells 40-57)
siteDistances(14:19) = 2*isd;
siteAngles(14:19) = 30:60:360;

% Initialize arrays for cell transmitter parameters
numCells = numCellSites*3;
cellLats = zeros(1,numCells);
cellLons = zeros(1,numCells);
cellNames = strings(1,numCells);
cellAngles = zeros(1,numCells);

% Define cell sector angles
cellSectorAngles = [30 150 270];

% For each cell site location, populate data for each cell transmitter
cellInd = 1;
for siteInd = 1:numCellSites
    % Compute site location using distance and angle from center site
    [cellLat,cellLon] = location(centerSite, siteDistances(siteInd), siteAngles(siteInd));
    
    % Assign values for each cell
    for cellSectorAngle = cellSectorAngles
        cellNames(cellInd) = "Cell " + cellInd;
        cellLats(cellInd) = cellLat;
        cellLons(cellInd) = cellLon;
        cellAngles(cellInd) = cellSectorAngle;
        cellInd = cellInd + 1;
    end
end





% Define transmitter parameters using Table 8-2 (b) of Report ITU-R M.[IMT-2020.EVAL]
% fq = 1.5e+9; % Carrier frequency (4 GHz) for Dense Urban-eMBB


antHeight = 25; % m
txPowerDBm = 44; % Total transmit power in dBm
txPower = 10.^((txPowerDBm-30)/10); % Convert dBm to W

ind = 1:60;
fq = zeros(length(ind));

fq(1) = 700e+6;
fq(2) = 900e+6;  
fq(3) = 1.5e+9;

perc_binRate_in_5G_std_speed = [];

for inx=1:3   
    
    
%     % Create cell transmitter sites
%     txs = txsite('Name',cellNames, ...
%         'Latitude',cellLats, ...
%         'Longitude',cellLons, ...
%         'ReceiverSensitivity', -100, ...
%         'AntennaAngle',cellAngles, ...
%         'AntennaHeight',antHeight, ...
%         'TransmitterFrequency',fq(i), ...
%         'TransmitterPower',txPower);

     % Create cell transmitter sites
    txs = txsite('Name',cellNames, ...
        'Latitude',cellLats, ...
        'Longitude',cellLons, ...
        'AntennaAngle',cellAngles, ...
        'AntennaHeight',antHeight, ...
        'TransmitterFrequency',fq(inx), ...
        'TransmitterPower',txPower);
    
    %figure(2)
    %coverage(txs,'close-in','SignalStrengths',-100:5:-60);
    
    
    
        % Launch Site Viewer
    viewer = siteviewer("Basemap","openstreetmap",...
       "Buildings","map.osm");

    % Show sites on a map
    show(txs);   
    viewer.Basemap = 'topographic';
    
    %Save SINR value in a variable
    %relSIN{i} = sinr(txs); 

    % Define pattern parameters
    azvec = -180:180;
    elvec = -90:90;
    Am = 10; % Maximum attenuation (dB)
    tilt = 0; % Tilt angle
    az3dB = 35; % 3 dB bandwidth in azimuth
    el3dB = 65; % 3 dB bandwidth in elevation

    % Define antenna pattern
    [az,el] = meshgrid(azvec,elvec);
    azMagPattern = -12*(az/az3dB).^2;
    elMagPattern = -12*((el-tilt)/el3dB).^2;
    combinedMagPattern = azMagPattern + elMagPattern;
    combinedMagPattern(combinedMagPattern<-Am) = -Am; % Saturate at max attenuation
    phasepattern = zeros(size(combinedMagPattern));

    % Create antenna element
    antennaElement = phased.CustomAntennaElement(...
        'AzimuthAngles',azvec, ...
        'ElevationAngles',elvec, ...
        'MagnitudePattern',combinedMagPattern, ...
        'PhasePattern',phasepattern);

    % Display radiation pattern
    f = figure;
    pattern(antennaElement,fq(inx));

    % Assign the antenna element for each cell transmitter
    for tx = txs
        tx.Antenna = antennaElement;
    end

    % Define receiver parameters using Table 8-2 (b) of Report ITU-R M.[IMT-2020.EVAL] 
    bw = 20e6; % 20 MHz bandwidth
    rxNoiseFigure = 7; % dB
    rxNoisePower = -174 + 10*log10(bw) + rxNoiseFigure;
    rxGain = 0; % dBi
    rxAntennaHeight = 1.5; % m

    % Display SINR map
    if isvalid(f)
        close(f)
    end
%     sinr(txs,'freespace', ...
%         'ReceiverGain',rxGain, ...
%         'ReceiverAntennaHeight',rxAntennaHeight, ...
%         'ReceiverNoisePower',rxNoisePower, ...    
%         'MaxRange',isd, ...
%         'Resolution',isd/20)
%     
    sinr(txs,'raytracing', ...
        'ReceiverGain',rxGain, ...
        'ReceiverAntennaHeight',rxAntennaHeight, ...
        'ReceiverNoisePower',rxNoisePower, ...    
        'MaxRange',isd/50, ...
        'Resolution',1)  %2km

    % Define array size
    nrow = 8;
    ncol = 8;

    %Define element spacing
    lambda = physconst('lightspeed')/fq(inx);
    drow = lambda/2;
    dcol = lambda/2;

    % Define taper to reduce sidelobes 
    dBdown = 30;
    taperz = chebwin(nrow,dBdown);
    tapery = chebwin(ncol,dBdown);
    tap = taperz*tapery.'; % Multiply vector tapers to get 8-by-8 taper values

    %Create 8-by-8 antenna array
    cellAntenna = phased.URA('Size',[nrow ncol], ...
        'Element',antennaElement, ...
        'ElementSpacing',[drow dcol], ...
        'Taper',tap, ...
        'ArrayNormal','x');

    % Display radiation pattern
    f = figure;
    pattern(cellAntenna,fq(inx));

    % Assign the antenna array for each cell transmitter, and apply downtilt.
    % Without downtilt, pattern is too narrow for transmitter vicinity.
    % downtilt = 15;
    % for tx = txs
    %     tx.Antenna = cellAntenna;
    %     tx.AntennaAngle = [tx.AntennaAngle; -downtilt];
    % end
    % 
    % % Display SINR map
    % if isvalid(f)
    %     close(f)
    % end
    % sinr(txs,'freespace', ...
    %     'ReceiverGain',rxGain, ...
    %     'ReceiverAntennaHeight',rxAntennaHeight, ...
    %     'ReceiverNoisePower',rxNoisePower, ...    
    %     'MaxRange',isd, ...
    %     'Resolution',isd/20)

    %------------------------------------------------------%

    % sinr(txs,'close-in', ...
    %     'ReceiverGain',rxGain, ...
    %     'ReceiverAntennaHeight',rxAntennaHeight, ...
    %     'ReceiverNoisePower',rxNoisePower, ...    
    %     'MaxRange',isd, ...
    %     'Resolution',isd/20)

    %------------------------------------------------------%

    %Design half-wavelength rectangular microstrip patch antenna
    patchElement = design(patchMicrostrip,fq(inx));
    patchElement.Width = patchElement.Length;
    patchElement.Tilt = 90;
    patchElement.TiltAxis = [0 1 0];

    % Display radiation pattern
    f = figure;
    pattern(patchElement,fq(inx))
    % 
    % %------------------------------------------------------%
    % 
    % % Assign the patch antenna as the array element
    % cellAntenna.Element = patchElement;
    % 
    % % Display SINR map
    % if isvalid(f)
    %     close(f)
    % end
    % sinr(txs,'close-in',...
    %     'ReceiverGain',rxGain, ...
    %     'ReceiverAntennaHeight',rxAntennaHeight, ...
    %     'ReceiverNoisePower',rxNoisePower, ...    
    %     'MaxRange',isd, ...
    %     'Resolution',isd/20)
    %------------------------------------------------------%
    
    %best_sinr(i) = max(relSIN{1, i}.Data{:, 3});
    
    %--------------------------------------------------------%
    
    
    %Cálculo da Taxa de Reutilização de Canal Comum
    q = sqrt(3*numCellSites);
    
    numChannels = 200;
    numberChannels_perSetor = 200/(numCellSites*3);
    per_congest = 1;  % 1%
    traf_per_set = 5.16;   %E
    traf_per_cell =traf_per_set*3;
    
    dens_pop_Leiria = 224.6;  %hab/km2
    
    num_users_per_cell = dens_pop_Leiria*((2*isd)/1000);
    A_utiliz = traf_per_cell/num_users_per_cell;
    A_utiliz_min = 60*A_utiliz;
    
   
        A_utiliz_sec = (A_utiliz_min-3)*60;
        A_utiliz_timeForCallInAnHour =[3 A_utiliz_sec];
        A_utiliz_min_day = (60*A_utiliz)*24;
        A_utiliz_timeForCall_day = [1 24 (A_utiliz_min_day-84)*60];
        A_utiliz_min_month = A_utiliz_min_day*30;
        A_utiliz_timeForCall_month = [1 18 8 0];
        A_utiliz_min_year = A_utiliz_min_month*12;
        A_utiliz_timeForCall_year = [21 4 0 0];
    
    
    rend_utiliz = (traf_per_cell/(numberChannels_perSetor*3))*100;
    %-------------------------------
    ad_point1(1) = (39.744 + 39.74578)/2;   %Latitude of Adicional Point 1
    ad_point1(2) = (-8.8065-8.81243)/2;     %Longitude of Adicional Point 1
    
    ad_point2(1) = (39.739704+39.743177)/2;
    ad_point2(2) = (-8.8025-8.8065)/2;
    
    latForT(1) = 39.744; 
    latForT(2) = 39.74578;
    latForT(3) = 39.7415;
    
    longForT(1) = -8.8065;  
    longForT(2) = -8.81243;
    longForT(3) = -8.812;   
    
    sumCoord_lat = 0;
    sumCoord_long = 0;
    
    for i=1:3
        sumCoord_lat = sumCoord_lat + latForT(i);
        sumCoord_long = sumCoord_long + longForT(i);
    end
    
    CentTriangle(1) = sumCoord_lat/3;
    CentTriangle(2) =  sumCoord_long/3;
    
    
    latForT_1(1) = 39.744; 
    latForT_1(2) = 39.747673;
    latForT_1(3) = 39.743177;
    
    longForT_1(1) = -8.8065;  
    longForT_1(2) = -8.802805;
    longForT_1(3) = -8.8025;
    
    sumCoord_1_lat = 0;
    sumCoord_1_long = 0;
    
    for i=1:3
        sumCoord_1_lat = sumCoord_1_lat + latForT_1(i);
        sumCoord_1_long = sumCoord_1_long + longForT_1(i);
    end
    
    CentTriangle_1(1) = sumCoord_1_lat/3;
    CentTriangle_1(2) =  sumCoord_1_long/3;
   
    
    %Falta colocar um no triângulo
    
    namesz = ['Zona Norte do Estádio Dr. Magalhães Pessoa', 'Zona Sul do Jardim Luís de Camões', 'Cemitério de Leiria', 'Place4', 'Place5', 'Place6', 'Place7', 'Zona Habitacional junto ao estádio', 'Zona Habitacional Oeste', 'Zona Habitacional Este', 'Extra 1', 'Center of Triangle 1', 'Center of Triangle 2', 'Center of Triangle 3'];  %Nomes dos pontos onde queremos fazer um estudo mais detalhado
    lats = [39.7495 39.744 39.745378 39.747 39.7415 39.743177 39.746 39.74578 ad_point1(1) ad_point2(1) 39.747673 CentTriangle(1) CentTriangle_1(1) 39.74331367];   %Latitudes dos pontos onde queremos fazer um estudo mais detalhado
    lons = [-8.8125 -8.8065 -8.80056 -8.809 -8.812 -8.8025 -8.8065 -8.81243 ad_point1(2) ad_point2(2) -8.802805 CentTriangle(2) CentTriangle_1(2) -8.8085];   %Longitudes dos pontos onde queremos fazer um estudo mais detalhado
    sens = -90;  %Sensibilidade dos recetores onde queremos fazer um estudo mais detalhado
    
    rxs = rxsite('Name', namesz,...
      'Antenna',dipole, 'Latitude',lats,...
       'Longitude',lons, ...
       'ReceiverSensitivity',sens);
    show(rxs)
    
    
    for ind = 1:length(lats)
        [d1km, d2_km]=distance(centerLocation,[lats(ind) lons(ind)]);
        
        vector_DistanceToCriticalPoints_t(ind) = d1km;
        
        vector_DistanceToCriticalPoints = 1000*vector_DistanceToCriticalPoints_t.';  %To metters
    end
    
%     [C, ia, ic] = uniquetol(vector_DistanceToCriticalPoints, 0.0001);
%     
%     distDiff = unique(C(ic));
    
    %result = findArrayOfPositionsWhereElementsAre(vector_DistanceToCriticalPoints);
    
    numCompSit = 2;
    b = [];
    
    t(1) = 1.0000;
    t(2) = 4.5829;  
    
    
    %---------------------------------------------------------------------------------------------------%
    tx_1 = txs(1,1);
    %---------------------------------------------------------------------------------------------------%
    tx_2 = txs(1,4);
    %---------------------------------------------------------------------------------------------------%
    tx_3 = txs(1,7);
    %---------------------------------------------------------------------------------------------------%
    tx_4 = txs(1,10);
    %---------------------------------------------------------------------------------------------------%
    tx_5 = txs(1,13);
    %---------------------------------------------------------------------------------------------------% 
    tx_6 = txs(1,16);
    %---------------------------------------------------------------------------------------------------% 
    pm = propagationModel("raytracing");
    
    
    ss(1) = sigstrength(rxs(1,5),tx_5);
    ss(2) = sigstrength(rxs(1,1),tx_4);
    ss(3) = sigstrength(rxs(1,2),tx_1);
    ss(4) = sigstrength(rxs(1,4),tx_1);
    ss(5) = sigstrength(rxs(1,3),tx_2);
    ss(6) = sigstrength(rxs(1,5),tx_2);
    ss(7) = sigstrength(rxs(1,7),tx_1);    
    ss(8) = sigstrength(rxs(1,8),tx_4);  % , pm
    ss(9) = sigstrength(rxs(1,9),tx_1);  % , pm
    ss(10) = sigstrength(rxs(1,10),tx_6);  %, pm
    ss(11) = sigstrength(rxs(1,11),tx_6);
    ss(12) = sigstrength(rxs(1,12),tx_5);
    ss(13) = sigstrength(rxs(1,13),tx_1);
    ss(14) = sigstrength(rxs(1,14),tx_1);
    
    
    
    max_perc_binRate_in_5G_std_speed = max(perc_binRate_in_5G_std_speed);
    
    nbins = 10;
    
    figure(5)
    
    subplot(2,1,1);
    h = histogram(ss,nbins)
    xlabel('SINR (dB)');
    ylabel('Number of Selected Targets');
    title('Análise da cobertura do nível de sinal');
    
    margin(1) = abs(rxs(1,5).ReceiverSensitivity - ss(1));
    margin(2) = abs(rxs(1,1).ReceiverSensitivity - ss(2));
    margin(3) = abs(rxs(1,2).ReceiverSensitivity - ss(3));
    margin(4) = abs(rxs(1,4).ReceiverSensitivity - ss(4));
    margin(5) = abs(rxs(1,3).ReceiverSensitivity - ss(5));
    margin(6) = abs(rxs(1,5).ReceiverSensitivity - ss(6));
    margin(7) = abs(rxs(1,7).ReceiverSensitivity - ss(7));
    margin(8) = abs(rxs(1,8).ReceiverSensitivity - ss(8));
    margin(9) = abs(rxs(1,9).ReceiverSensitivity - ss(9));
    margin(10) = abs(rxs(1,10).ReceiverSensitivity - ss(10));
    margin(11) = abs(rxs(1,11).ReceiverSensitivity - ss(11));
    margin(12) = abs(rxs(1,12).ReceiverSensitivity - ss(12));
    margin(13) = abs(rxs(1,13).ReceiverSensitivity - ss(13));
    margin(14) = abs(rxs(1,14).ReceiverSensitivity - ss(14));
    
    subplot(2,1,2);
    
    h_m = histogram(margin,nbins)
    xlabel('SNIR (dB)');
    ylabel('Number of Selected Targets');
    title('Análise da margem de manobra nos recetores');
    
    sum_ss = 0;
    sum_margin = 0;
    
    Shannon_limit = [];
    bandwidth = 100e+6;   %fq(inx)
    
    ant_mimo = 1;  %6
    
    for num=1:length(ss)
        Shannon_limit(num) = bandwidth*log2(1+ant_mimo*10^(margin(num)/10));
    end
    
    figure(4)
    plot(1:length(ss), Shannon_limit)
    xlabel('Zone Number')
    ylabel('Maximum Bit Rate (bps)')
    title('Shannon Limit Bit Rate')
    
    higherRateLimitShannon = max(Shannon_limit);
    
    
    download_5G_std_speed = 10.240e+9;  
    upload_5G_std_speed = 2*download_5G_std_speed;
    
    if higherRateLimitShannon < download_5G_std_speed
        disp('Débito binário não conseguido')
        perc_binRate_in_5G_std_speed(inx) = (higherRateLimitShannon/download_5G_std_speed)*100;
        disp('A utilização do handover é necessária, dado que o estudo está a ser feito, para uma probabilidade de congestão de 1%')
    else
        disp('Débito binário conseguido')
        margemManobra_deb_bin = higherRateLimitShannon-download_5G_std_speed;
        pointsWithNotEnoughBinaryRate = find(Shannon_limit<download_5G_std_speed);
        disp('Não é absolutamente necessária a utilização do handover')
    end
    
    for pointsOfStudy=1:length(ss)
        sum_ss = sum_ss + ss(pointsOfStudy);
        sum_margin = sum_margin + margin(pointsOfStudy);
    end
    
    av_ss = sum_ss/length(ss);
    av_margin = sum_margin/length(margin);
    %--------------------------------------------------------------------------------------------------%
    
    bestServer_sit = [];
    
    loss = [];
    
    for ps=1:length(ss)
        loss(ps) = txPowerDBm - ss(ps);
    end
    
    
    for n=1:numCompSit
        a = (find((abs(vector_DistanceToCriticalPoints - t(n)))<0.1)).';
        
%        if n==2
%            a = cat(2,a,zeros(1,1));
%        end

        bestServer_sit(n) = findBestServer(ss(a(1)), ss(a(2)));
        if(bestServer_sit(n) == 0)
            bestServer_sit(n) = ss(a(1));
            disp('Igual nível de sinal. Ambos representam o Best Server')
        end    
        ss_bs = find(ss == bestServer_sit(n));
        
        %placeBestServer(1) = char(namesz{ss_bs(1)});
        
        latBestServer(n) = lats(ss_bs(1));
        longBestServer(n) = lons(ss_bs(1));
        
        b = cat(1,b,a);
        %b(n,:) = a;
    end
    
    bestOfBestServers(1) = max(latBestServer);
    bestOfBestServers(2) = max(longBestServer);
    
    
    
    %-----------------------------------------------------------------------------------21_6-------------------------------
    % For each cell site location, populate data for each cell transmitter
    
%     siteDistancesIndStudy = [0 5 5 5 5 5 5 10*cosd(30) 10*cosd(30) 10*cosd(30) 10*cosd(30) 10*cosd(30) 10*cosd(30) 10 10 10 10 10 10];
%     cellInd = 1;
%     for siteInd = 1:numCellSites
%         % Compute site location using distance and angle from center site
%         [cellLatIndStudy,cellLonIndStudy] = location(rxs, siteDistancesIndStudy(siteInd), siteAngles(siteInd));
% 
%         % Assign values for each cell
%         for cellSectorAngle = cellSectorAngles
%             cellNamesIndStudy(cellInd) = "Cell " + cellInd;
%             cellLatsIndStudy(cellInd) = cellLat;
%             cellLonsIndStudy(cellInd) = cellLon;
%             cellAnglesIndStudy(cellInd) = cellSectorAngle;
%             cellInd = cellInd + 1;
%         end
%     end
    
    %-----------------------------------------------------------------------------------21_6-------------------------------

    
    %--------------------------------------------------------%
    %SNIR = getSNIR(39.739566, -8.885913);
    
    %break;
    
    if inx==1
        S_1 = struct('Sinal_Strength',num2cell(ss),'Margem_de_Manobra',num2cell(margin),'Shannon_Limit',num2cell(Shannon_limit), 'Best_of_best_servers', num2cell(cat(2,bestOfBestServers,zeros(1,12))), 'Loss', loss);
        for indrx=1:length(ss)
            kmlwritepoint('studyKPI_700MHz.kml', lats(indrx), lons(indrx));
        end
    else if inx == 2
            S_2 = struct('Sinal_Strength',num2cell(ss),'Margem_de_Manobra',num2cell(margin),'Shannon_Limit',num2cell(Shannon_limit), 'Best_of_best_servers', num2cell(cat(2,bestOfBestServers,zeros(1,12))), 'Loss', loss);
        
            for indrx=1:length(ss)
                kmlwritepoint('studyKPI_900MHz.kml', lats(indrx), lons(indrx));
            end
        else
            S_3 = struct('Sinal_Strength',num2cell(ss),'Margem_de_Manobra',num2cell(margin),'Shannon_Limit',num2cell(Shannon_limit), 'Best_of_best_servers', num2cell(cat(2,bestOfBestServers,zeros(1,12))), 'Loss', loss);
            
            for indrx=1:length(ss)
                kmlwritepoint('studyKPI_1500MHz.kml', lats(indrx), lons(indrx));
            end
        end
    end
    
    %S_1 = struct('Sinal_Strength',num2cell(ss),'Margem_de_Manobra',num2cell(margin),'Shannon_Limit',num2cell(Shannon_limit), 'Best_of_best_servers', num2cell(cat(2,bestOfBestServers,zeros(1,12))));
    
    
    %clf
    
    %break;
end

save CM_Project.mat S_1 S_2 S_3

%sinrBestServer = max(best_sinr);

function sinalStrengthBestServer = findBestServer(ss_1, ss_2)
    
    if ss_1>ss_2
        sinalStrengthBestServer = ss_1;
    else if ss_2>ss_1
            sinalStrengthBestServer = ss_2;
        else
            sinalStrengthBestServer = 0;
        end
    end

end

function res = findArrayOfPositionsWhereElementsAre(d)
    [C, ia, ic] = uniquetol(d, 0.0001);
    
    distDiff = unique(C(ic));
     
    len_max = 0;
%     for i = 1:length(distDiff)
%         if length(find(d == distDiff(i))) > len_max
%             len_max = length(find(d == distDiff(i)));
%         end
%     end

    th = [1.0000 0 0].';
    tol = 0.0001;
    for ind = 1:length(th)
         
         [ii,jj]=find(abs(d-th(ind))<tol);
         if (size(ii) > len_max)
             len_max_vec = size(ii);
         end        
       
    end
    
    len_max = max(len_max_vec);

    res = [];

    for i = 1:length(distDiff)
    %for i = 1:length(d)
        if(len_max > 1)
            
            x = distDiff(i);
            %l = length(find(d == x));
            sum_l = 0;
            for indi=1:length(d)
                l = length(find(abs(d(indi) - x) == 0));
                sum_l = sum_l + l;
            end
            
            l_vector(i) = sum_l;
            
            
            if length(find(d == x)) ~= len_max
                parcel = cat(2, find(d == distDiff(i)), zeros(1, len_max - length(find(d == distDiff(i)))));
                parcel_VEC(i,:) = parcel;
                res = cat(1, res, parcel);
            else
                [M,F] = mode(d(:));
                res = cat(1, res, find(d == M)); 
            end
        else 
            parcel = cat(2, find(d == distDiff(i)), zeros(1, len_max - length(find(d == distDiff(i)))));
            parcel_VEC(i,:) = parcel;
            res = cat(1, res, parcel);
        end
    end    
     
end



function snir_table = getSNIR(lat, long)
    antHeight = 25; % m
    txPowerDBm = 44; % Total transmit power in dBm
    txPower = 10.^((txPowerDBm-30)/10); % Convert dBm to W



    % Define center location site (cells 1-3)
    centerSite = txsite('Name','MathWorks Glasgow', ...
        'Latitude',lat,...
        'Longitude',long);

    % Initialize arrays for distance and angle from center location to each cell site, where
    % each site has 3 cells
    numCellSites = 6;
    siteDistances = zeros(1,numCellSites);
    siteAngles = zeros(1,numCellSites);

    % Define distance and angle for inner ring of 6 sites (cells 4-21)
    isd = 20/cosd(30); % Inter-site distance   % 500/cosd(30)
    siteDistances(2:7) = isd;
    siteAngles(2:7) = 30:60:360;

    % Define distance and angle for middle ring of 6 sites (cells 22-39)
    siteDistances(8:13) = 2*isd*cosd(30);
    siteAngles(8:13) = 0:60:300;

    % Define distance and angle for outer ring of 6 sites (cells 40-57)
    siteDistances(14:19) = 2*isd;
    siteAngles(14:19) = 30:60:360;

    % Initialize arrays for cell transmitter parameters
    numCells = numCellSites*3;
    cellLats = zeros(1,numCells);
    cellLons = zeros(1,numCells);
    cellNames = strings(1,numCells);
    cellAngles = zeros(1,numCells);

    % Define cell sector angles
    cellSectorAngles = [30 150 270];

    % For each cell site location, populate data for each cell transmitter
    cellInd = 1;
    for siteInd = 1:numCellSites
        % Compute site location using distance and angle from center site
        [cellLat,cellLon] = location(centerSite, siteDistances(siteInd), siteAngles(siteInd));

        % Assign values for each cell
        for cellSectorAngle = cellSectorAngles
            cellNames(cellInd) = "Cell " + cellInd;
            cellLats(cellInd) = cellLat;
            cellLons(cellInd) = cellLon;
            cellAngles(cellInd) = cellSectorAngle;
            cellInd = cellInd + 1;
        end
    end
    
    txsd = txsite('Name',cellNames, ...
        'Latitude',cellLats, ...
        'Longitude',cellLons, ...
        'AntennaAngle',cellAngles, ...
        'AntennaHeight',antHeight, ...
        'TransmitterFrequency',100e+6, ...
        'TransmitterPower',txPower);
    
    
    snir_table = sinr(txsd);
end