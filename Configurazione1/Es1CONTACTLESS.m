percorso_slave = '/Users/lucacampodonico/Downloads/20260417_110546/117322072431_pointcloud_1.ply'; 
percorso_master = '/Users/lucacampodonico/Downloads/20260417_110546/116622071830_pointcloud_1.ply';

ptCloudMaster = pcread(percorso_master);
ptCloudSlave = pcread(percorso_slave);

figure('Name', 'Step 1: Verifica Orientamento');
subplot(1, 2, 1);
pcshow(ptCloudMaster);
title('Nuvola Master');
xlabel('X'); ylabel('Y'); zlabel('Z');

subplot(1, 2, 2);
pcshow(ptCloudSlave);
title('Nuvola Slave');
xlabel('X'); ylabel('Y'); zlabel('Z');

larghezza = 640;
altezza = 480;

clear; clc; close all;

nome_file = '/Users/lucacampodonico/Downloads/20260417_110546/117322072431_pointcloud_1.ply'; 

fx = 600; 
fy = 600; 
ox = 320; 
oy = 240; 

img_width = 640;  
img_height = 480; 

ptCloud = pcread(nome_file);

XYZ = ptCloud.Location;
X = XYZ(:, 1);
Y = XYZ(:, 2);
Z = XYZ(:, 3);
colori = ptCloud.Color; 

Y = -Y;
Z = -Z;

depth_map = NaN(img_height, img_width); 
num_canali = size(colori, 2);
color_img_convertita = zeros(img_height, img_width, num_canali, 'uint8');

i = round(fx .* (X ./ Z) + ox) + 1; 
j = round(fy .* (Y ./ Z) + oy) + 1; 

punti_validi = (i >= 1) & (i <= img_width) & ...
               (j >= 1) & (j <= img_height) & ...
               (Z > 0);

for k = 1:length(punti_validi)
    if punti_validi(k)
        colonna = i(k);
        riga = j(k);
        
        depth_map(riga, colonna) = Z(k);
        color_img_convertita(riga, colonna, :) = colori(k, :);
    end
end

figure('Name', 'Risultato Punto 3.a', 'Position', [100, 100, 900, 400]);
subplot(1, 2, 1);
imagesc(depth_map);
axis image; colorbar; colormap jet;
title(sprintf('Depth Map\nDimensione: %dx%d', size(depth_map,1), size(depth_map,2)));

subplot(1, 2, 2);
imshow(color_img_convertita);
title(sprintf('Immagine Convertita\nDimensione: %dx%d', size(color_img_convertita,1), size(color_img_convertita,2)));

R = [-0.40491282, 0.68898847, -0.60111605, 0;
 -0.75778142 , 0.11503075 , 0.64228907, 0;
  0.51167659 , 0.71558565 , 0.47552523, 0];

T = [0.58126756, -0.50985878 , 0.35201821, 1]; 

tform = rigidtform3d(R, T);

ptCloudSlaveAligned = pctransform(ptCloudSlave, tform);

figure('Name', 'Punto 3.d: Allineamento Nuvole', 'Position', [150, 150, 800, 600]);
pcshow(ptCloudMaster);
hold on; 
pcshow(ptCloudSlaveAligned);
hold off;

title('Sovrapposizione: Master e Slave Allineata');
xlabel('X'); ylabel('Y'); zlabel('Z');

disp('Trasformazione rigida applicata con successo.');
disp('Verifica visivamente se le due nuvole sono allineate correttamente.');