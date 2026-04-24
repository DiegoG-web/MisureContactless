%% INIZIALIZZAZIONE E CARICAMENTO
clear; clc; close all;

% Percorsi dei file (assicurati che puntino ai file corretti)
percorso_slave = '/Users/lucacampodonico/Desktop/Uni/CONTACTLESS/MisureContactless/Configurazione2/20260424_110710/116622071830_pointcloud_1.ply'; 
percorso_master = '/Users/lucacampodonico/Desktop/Uni/CONTACTLESS/MisureContactless/Configurazione2/20260424_110710/117322072431_pointcloud_1.ply';

% Lettura delle Point Cloud
ptCloudMaster = pcread(percorso_master); % 1173...
ptCloudSlave = pcread(percorso_slave);   % 1166...

%% STEP 1: VERIFICA ORIENTAMENTO
figure('Name', 'Step 1: Verifica Orientamento', 'Position', [100, 500, 800, 400]);
subplot(1, 2, 1);
pcshow(ptCloudMaster); title('Nuvola Master Originale'); xlabel('X'); ylabel('Y'); zlabel('Z');
subplot(1, 2, 2);
pcshow(ptCloudSlave); title('Nuvola Slave Originale'); xlabel('X'); ylabel('Y'); zlabel('Z');

%% STEP 3.a: DA 3D A 2D (Depth Map e Immagine Convertita)
% Dati INTRINSECI esatti estratti dalla calibrazione della Slave (116622071830)
fx = 650.25690634; 
fy = 652.20427960; 
ox = 267.04769936; 
oy = 191.18846340; 
img_width = 640;  
img_height = 480; 

XYZ = ptCloudSlave.Location;
X = XYZ(:, 1);
Y = XYZ(:, 2);
Z = XYZ(:, 3);
colori = ptCloudSlave.Color; 

% Correzione orientamento visivo
Y_plot = -Y;
Z_plot = -Z;

depth_map = NaN(img_height, img_width); 
num_canali = size(colori, 2);
color_img_convertita = zeros(img_height, img_width, num_canali, 'uint8');

i = round(fx .* (X ./ Z_plot) + ox) + 1; 
j = round(fy .* (Y_plot ./ Z_plot) + oy) + 1; 
punti_validi = (i >= 1) & (i <= img_width) & (j >= 1) & (j <= img_height) & (Z_plot > 0);

for k = 1:length(punti_validi)
    if punti_validi(k)
        depth_map(j(k), i(k)) = Z_plot(k);
        color_img_convertita(j(k), i(k), :) = colori(k, :);
    end
end

figure('Name', 'Step 3.a: Mappe 2D', 'Position', [150, 100, 900, 400]);
subplot(1, 2, 1);
imagesc(depth_map); axis image; colorbar; colormap jet;
title(sprintf('Depth Map\nDim: %dx%d', size(depth_map,1), size(depth_map,2)));
subplot(1, 2, 2);
imshow(color_img_convertita);
title(sprintf('Immagine Convertita\nDim: %dx%d', size(color_img_convertita,1), size(color_img_convertita,2)));

%% STEP 3.d: TRASFORMAZIONE (Applicazione calibrazione Python)
% Dati ESTRINSECI esatti da 1166... a 1173...
R_cv = [ 0.69842453,  0.33521783, -0.63232285;
        -0.39555964,  0.91711687,  0.04928712;
         0.59643587,  0.21569806,  0.77313556];
          
T_cv = [ 0.46267045, -0.00002830,  0.08089408]; 

% 1. Forziamo la matrice a essere perfettamente ortogonale (buona pratica MATLAB)
[U, ~, V] = svd(R_cv);
R_ortho = U * V';

% 2. Creiamo la trasformazione. 
% NOTA BENE: Essendo già Slave -> Master, NON usiamo 'invert()'. 
% Trasponiamo solo R (R_ortho') perché MATLAB usa la convenzione opposta a Python.
tform_diretta = rigidtform3d(R_ortho', T_cv);

% 3. Applichiamo la trasformazione alla nuvola Slave
ptCloudSlaveAligned = pctransform(ptCloudSlave, tform_diretta);

%% STEP 5: ISOLAMENTO ROI E ICP DEFINITIVO (Affinamento)
disp('1. Avvicinamento globale (baricentri)...');
centro_Master = mean(ptCloudMaster.Location, 'omitnan');
centro_Slave = mean(ptCloudSlaveAligned.Location, 'omitnan');
spostamento = centro_Master - centro_Slave;
ptCloudSlaveVicino = pointCloud(ptCloudSlaveAligned.Location + spostamento, 'Color', ptCloudSlaveAligned.Color);

disp('2. Isolamento della Region of Interest (ROI)...');
dim_box = 0.25; % Raggio del cubo (25 cm - aggiustabile)
roi = [centro_Master(1)-dim_box, centro_Master(1)+dim_box, ...
       centro_Master(2)-dim_box, centro_Master(2)+dim_box, ...
       centro_Master(3)-dim_box, centro_Master(3)+dim_box];

ind_M = findPointsInROI(ptCloudMaster, roi);
ind_S = findPointsInROI(ptCloudSlaveVicino, roi);
pc_Master_ROI = select(ptCloudMaster, ind_M);
pc_Slave_ROI = select(ptCloudSlaveVicino, ind_S);

disp('3. Calcolo ICP concentrato...');
[tform_icp, ~] = pcregistericp(pc_Slave_ROI, pc_Master_ROI, ...
    'MaxIterations', 100, ...
    'Tolerance', [1e-5, 1e-5]);

disp('4. Applicazione dell''incastro a tutta la scena...');
ptCloudSlave_Finale = pctransform(ptCloudSlaveVicino, tform_icp);

% --- VISUALIZZAZIONE FINALE ---
figure('Name', 'Step 5: Fusione Finale', 'Position', [200, 200, 1000, 500]);
subplot(1, 2, 1);
pcshow(ptCloudMaster); hold on; pcshow(ptCloudSlaveVicino); hold off;
title('1. Pre-ICP (Solo calibrazione globale)'); xlabel('X'); ylabel('Y'); zlabel('Z');

subplot(1, 2, 2);
pcshow(ptCloudMaster); hold on; pcshow(ptCloudSlave_Finale); hold off;
title('2. Post-ICP (Allineamento fine)'); xlabel('X'); ylabel('Y'); zlabel('Z');

disp('Operazione completata! Controlla la qualità della fusione nel riquadro di destra.');












%% STEP 5: ISOLAMENTO ROI E ICP DEFINITIVO (Affinamento)
clear; clc; close all;

percorso_slave = '/Users/lucacampodonico/Desktop/Uni/CONTACTLESS/MisureContactless/Configurazione2/20260424_110710/116622071830_pointcloud_1.ply'; 
percorso_master = '/Users/lucacampodonico/Desktop/Uni/CONTACTLESS/MisureContactless/Configurazione2/20260424_110710/117322072431_pointcloud_1.ply';

ptCloudMaster = pcread(percorso_master); 
ptCloudSlave = pcread(percorso_slave);   

figure('Name', 'Step 1: Verifica Orientamento', 'Position', [100, 500, 800, 400]);
subplot(1, 2, 1);
pcshow(ptCloudMaster); title('Nuvola Master Originale'); xlabel('X'); ylabel('Y'); zlabel('Z');
subplot(1, 2, 2);
pcshow(ptCloudSlave); title('Nuvola Slave Originale'); xlabel('X'); ylabel('Y'); zlabel('Z');

fx = 650.25690634; 
fy = 652.20427960; 
ox = 267.04769936; 
oy = 191.18846340; 
img_width = 640;  
img_height = 480; 

XYZ = ptCloudSlave.Location;
X = XYZ(:, 1);
Y = XYZ(:, 2);
Z = XYZ(:, 3);
colori = ptCloudSlave.Color; 

Y_plot = -Y;
Z_plot = -Z;

depth_map = NaN(img_height, img_width); 
num_canali = size(colori, 2);
color_img_convertita = zeros(img_height, img_width, num_canali, 'uint8');

i = round(fx .* (X ./ Z_plot) + ox) + 1; 
j = round(fy .* (Y_plot ./ Z_plot) + oy) + 1; 
punti_validi = (i >= 1) & (i <= img_width) & (j >= 1) & (j <= img_height) & (Z_plot > 0);

for k = 1:length(punti_validi)
    if punti_validi(k)
        depth_map(j(k), i(k)) = Z_plot(k);
        color_img_convertita(j(k), i(k), :) = colori(k, :);
    end
end

figure('Name', 'Step 3.a: Mappe 2D', 'Position', [150, 100, 900, 400]);
subplot(1, 2, 1);
imagesc(depth_map); axis image; colorbar; colormap jet;
title(sprintf('Depth Map\nDim: %dx%d', size(depth_map,1), size(depth_map,2)));
subplot(1, 2, 2);
imshow(color_img_convertita);
title(sprintf('Immagine Convertita\nDim: %dx%d', size(color_img_convertita,1), size(color_img_convertita,2)));

R_cv = [ 0.69842453,  0.33521783, -0.63232285;
        -0.39555964,  0.91711687,  0.04928712;
         0.59643587,  0.21569806,  0.77313556];
          
T_cv = [ 0.46267045, -0.00002830,  0.08089408]; 

[U, ~, V] = svd(R_cv);
R_ortho = U * V';

tform_diretta = rigidtform3d(R_ortho', T_cv);
ptCloudSlaveAligned = pctransform(ptCloudSlave, tform_diretta);

disp('1. Isolamento della Region of Interest (ROI)...');

punti_vicini = ptCloudMaster.Location(ptCloudMaster.Location(:,3) < 0.6, :);
centro_statua = mean(punti_vicini, 'omitnan');

dim_box = 0.15; 
roi = [centro_statua(1)-dim_box, centro_statua(1)+dim_box, ...
       centro_statua(2)-dim_box, centro_statua(2)+dim_box, ...
       centro_statua(3)-dim_box, centro_statua(3)+dim_box];

ind_M = findPointsInROI(ptCloudMaster, roi);
ind_S = findPointsInROI(ptCloudSlaveAligned, roi); 

pc_Master_ROI = select(ptCloudMaster, ind_M);
pc_Slave_ROI = select(ptCloudSlaveAligned, ind_S);

disp('2. Calcolo ICP concentrato...');
[tform_icp, ~] = pcregistericp(pc_Slave_ROI, pc_Master_ROI, ...
    'MaxIterations', 100, ...
    'Tolerance', [1e-5, 1e-5]);

disp('3. Applicazione dell''incastro a tutta la scena...');
ptCloudSlave_Finale = pctransform(ptCloudSlaveAligned, tform_icp);

figure('Name', 'Step 5: Fusione Finale', 'Position', [200, 200, 1000, 500]);
subplot(1, 2, 1);
pcshow(ptCloudMaster); hold on; pcshow(ptCloudSlaveAligned); hold off;
title('1. Pre-ICP (Solo calibrazione Python)'); xlabel('X'); ylabel('Y'); zlabel('Z');

subplot(1, 2, 2);
pcshow(ptCloudMaster); hold on; pcshow(ptCloudSlave_Finale); hold off;
title('2. Post-ICP (Allineamento fine statua)'); xlabel('X'); ylabel('Y'); zlabel('Z');

disp('Operazione completata!');