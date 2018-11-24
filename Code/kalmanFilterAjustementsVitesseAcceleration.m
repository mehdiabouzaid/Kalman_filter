function y=kalmanFilterAjustementsVitesseAcceleration(z, message)

delta_t=1/30;

% Matrice de dynamique/transfert/systeme
A=[1 0 delta_t 0 0 0;...     % x
   0 1 0 delta_t 0 0;...     % y
   0 0 1 0 delta_t 0;...     % Vx
   0 0 0 1 0 delta_t;...     % Vy
   0 0 0 0 1 0 ;...          % Ax
   0 0 0 0 0 1];             % Ay

% Matrice d'observation
H=[1 0 0 0 0 0; 0 1 0 0 0 0];

% Bruit de transition
Q=100000*diag([25 25 10 10 1 1]);

% Bruit d'observation
R=25*eye(2);

% Une variable persistent est locale a la fonction mais sa valeur est
% conservee entre appels de fonction
persistent x_estime p_estime
if isempty(x_estime)
    x_estime=zeros(6, 1);     % x_estime=[x,y,Vx,Vy,Ax,Ay]'
    p_estime=100000*eye(6);   % covariance de l'etat initial
    x_estime(1:2)=z;          % position initiale
end

% Vitesse de l'objet (deux images suffisent) : v=dx/dt
persistent countFrame X0 X1 Y0 Y1 VX0 VY0 AX AY VX0stocke VY0stocke;

if isempty(countFrame)
    countFrame=1;
else
    countFrame=countFrame+1;
end

if (strcmp(message, 'objet detecte')==1)
    if (countFrame==1)
        X0=z(1);
        Y0=z(2);
    end

    if (countFrame>=2)
        X1=z(1);
        Y1=z(2);
        if (countFrame<=3)
            VX0=(X1-X0)/(delta_t);
            VY0=(Y1-Y0)/(delta_t);
            VX0stocke=VX0;
            VY0stocke=VY0;
        else
            VX0=((X1-X0)/(delta_t))+delta_t*AX;
            VY0=((Y1-Y0)/(delta_t))+delta_t*AY;
        end

        x_estime(3:4)=[VX0; VY0];
        X0=X1;
        Y0=Y1;    

        % Acceleration (on a besoin de trois images) : a=dv/dt
        if (countFrame==3)
            AX=(VX0-VX0stocke)/(delta_t);
            AY=(VY0-VY0stocke)/(delta_t);
            x_estime(5:6)=[AX; AY];
        end
    end
end

% Prediction
x_predit=A*x_estime;
p_predit=A*p_estime*A'+Q;

if (strcmp(message, 'objet detecte')==1)
    % Mise a jour
    v=z-H*x_predit;
    S=H*p_predit*H'+R;
    K=(S\(H*p_predit'))';

    x_estime=x_predit+K*v;
    p_estime=p_predit-K*S*K';
else % strcmp(message, 'objet manquant')==1
	x_estime=x_predit;
	p_estime=p_predit;
end

% Estimation des observations
y=H*x_estime;

end
