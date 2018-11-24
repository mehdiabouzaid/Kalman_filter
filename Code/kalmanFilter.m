function y=kalmanFilter(z, delta_t, message)

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