%% INIZIALIZZAZIONE E CARICAMENTO
clear; clc; close all;

% Percorsi dei file
percorso_slave = '/Users/lucacampodonico/Downloads/20260417_110546/117322072431_pointcloud_1.ply'; 
percorso_master = '/Users/lucacampodonico/Downloads/20260417_110546/116622071830_pointcloud_1.ply';

% Lettura delle Point Cloud
ptCloudMaster = pcread(percorso_master);
ptCloudSlave = pcread(percorso_slave);

%% STEP 1: VERIFICA ORIENTAMENTO
figure('Name', 'Step 1: Verifica Orientamento', 'Position', [100, 500, 800, 400]);
subplot(1, 2, 1);
pcshow(ptCloudMaster);
title('Nuvola Master Originale');
xlabel('X'); ylabel('Y'); zlabel('Z');

subplot(1, 2, 2);
pcshow(ptCloudSlave);
title('Nuvola Slave Originale');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% STEP 3.a: DA 3D A 2D (Depth Map e Immagine Convertita)
% Usiamo i dati della Nuvola Slave (come nel tuo codice originale)
fx = 600; 
fy = 600; 
ox = 320; 
oy = 240; 
img_width = 640;  
img_height = 480; 

XYZ = ptCloudSlave.Location;
X = XYZ(:, 1);
Y = XYZ(:, 2);
Z = XYZ(:, 3);
colori = ptCloudSlave.Color; 

% Correzione orientamento visivo
Y = -Y;
Z = -Z;

depth_map = NaN(img_height, img_width); 
num_canali = size(colori, 2);
color_img_convertita = zeros(img_height, img_width, num_canali, 'uint8');

i = round(fx .* (X ./ Z) + ox) + 1; 
j = round(fy .* (Y ./ Z) + oy) + 1; 

punti_validi = (i >= 1) & (i <= img_width) & (j >= 1) & (j <= img_height) & (Z > 0);

for k = 1:length(punti_validi)
    if punti_validi(k)
        colonna = i(k);
        riga = j(k);
        depth_map(riga, colonna) = Z(k);
        color_img_convertita(riga, colonna, :) = colori(k, :);
    end
end

figure('Name', 'Step 3.a: Mappe 2D', 'Position', [150, 100, 900, 400]);
subplot(1, 2, 1);
imagesc(depth_map);
axis image; colorbar; colormap jet;
title(sprintf('Depth Map\\nDimensione: %dx%d', size(depth_map,1), size(depth_map,2)));

subplot(1, 2, 2);
imshow(color_img_convertita);
title(sprintf('Immagine Convertita\\nDimensione: %dx%d', size(color_img_convertita,1), size(color_img_convertita,2)));

%% STEP 3.d: TRASFORMAZIONE INVERSA (Il segreto per la posizione a L)
R_python = [-0.40491282,  0.68898847, -0.60111605;
            -0.75778142,  0.11503075,  0.64228907;
             0.51167659,  0.71558565,  0.47552523];
          
T_python = [0.58126756, -0.50985878, 0.35201821]; 

% Forziamo la matrice a essere ortogonale per MATLAB
[U, ~, V] = svd(R_python);
R_perfetta = U * V';

% 1. Creiamo la trasformazione nominale (Master -> Slave)
% NOTA BENE: Ci va l'apice su R_perfetta per trasporla (convenzione MATLAB)
tform_python = rigidtform3d(R_perfetta', T_python);

% 2. LA MAGIA: INVERTIAMO LA TRASFORMAZIONE (Slave -> Master)
% Questo dice a MATLAB di percorrere la strada al contrario!
tform_corretta = invert(tform_python);

% 3. Applichiamo la trasformazione corretta alla nuvola Slave
ptCloudSlaveAligned = pctransform(ptCloudSlave, tform_corretta);

%% STEP 5: ISOLAMENTO ROI E ICP DEFINITIVO (Incastro della testa)
disp('1. Avvicinamento globale (baricentri)...');
centro_Master = mean(ptCloudMaster.Location, 'omitnan');
centro_Slave = mean(ptCloudSlaveAligned.Location, 'omitnan');
spostamento = centro_Master - centro_Slave;
ptCloudSlaveVicino = pointCloud(ptCloudSlaveAligned.Location + spostamento, 'Color', ptCloudSlaveAligned.Color);

disp('2. Isolamento della testa dal cartone (ROI)...');
% Il cartone fa da "ancora" perché ha troppi punti. Lo ignoriamo temporaneamente.
% Creiamo un "cubo" virtuale (circa 40 cm) attorno al baricentro per prendere solo la statua.
dim_box = 0.2; % Raggio del cubo (modifica questo valore se taglia troppa testa o prende troppo cartone)
roi = [centro_Master(1)-dim_box, centro_Master(1)+dim_box, ...
       centro_Master(2)-dim_box, centro_Master(2)+dim_box, ...
       centro_Master(3)-dim_box, centro_Master(3)+dim_box];

% Estraiamo SOLO i punti dentro questo cubo
ind_M = findPointsInROI(ptCloudMaster, roi);
ind_S = findPointsInROI(ptCloudSlaveVicino, roi);
pc_Master_SoloTesta = select(ptCloudMaster, ind_M);
pc_Slave_SoloTesta = select(ptCloudSlaveVicino, ind_S);

disp('3. Calcolo ICP concentrato solo sulla statua...');
% Ora l'ICP è obbligato a guardare solo i dettagli del viso!
[tform_icp, ~] = pcregistericp(pc_Slave_SoloTesta, pc_Master_SoloTesta, ...
    'MaxIterations', 100, ...
    'Tolerance', [1e-5, 1e-5]);

disp('4. Applicazione dell''incastro a tutta la scena...');
% Applichiamo la micro-trasformazione perfetta trovata sulla testa a tutta la scena intera
ptCloudSlave_Finale = pctransform(ptCloudSlaveVicino, tform_icp);

% --- VISUALIZZAZIONE FINALE ---
figure('Name', 'Step 5: Fusione Finale (Ottimizzata per la testa)', 'Position', [200, 200, 1000, 500]);

subplot(1, 2, 1);
pcshow(ptCloudMaster); hold on; pcshow(ptCloudSlaveVicino); hold off;
title('1. Pre-ICP (Cartone unito, testa sfasata)');
xlabel('X'); ylabel('Y'); zlabel('Z');

subplot(1, 2, 2);
pcshow(ptCloudMaster); hold on; pcshow(ptCloudSlave_Finale); hold off;
title('2. Post-ICP (Testa incollata perfettamente)');
xlabel('X'); ylabel('Y'); zlabel('Z');

disp('Operazione completata! Guarda la testa nel secondo riquadro.');