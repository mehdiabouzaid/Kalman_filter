function Matrice_y=trackingObjet(cell_positions, delta_t)

Matrice_y=[]; % contiendra les positions corrigees et les predictions pour les points non detectes
objetDetectePremiereFois=false;

for i=1:length(cell_positions)
    if ~isempty(cell_positions{i})
        objetDetectePremiereFois=true;
        z=cell_positions{i}';
        y=kalmanFilter(z, delta_t,'objet detecte');   % prediction/correction de la position detectee avec un filtre de Kalman
        %y=kalmanFilterAjustementsVitesseAcceleration(z, 'objet detecte');
        Matrice_y=[Matrice_y y];
    else if (objetDetectePremiereFois==true)
        y=kalmanFilter(y, delta_t,'objet manquant'); % prediction de la position non detectee avec un filtre de Kalman
        %y=kalmanFilterAjustementsVitesseAcceleration(y, 'objet manquant');
        Matrice_y=[Matrice_y y];
        end
    end
end

Matrice_y=Matrice_y';

end