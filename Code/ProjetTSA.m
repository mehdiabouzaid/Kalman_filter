%%
clear all
clc
close all

%% Choix de la video a etudier

path=which('ProjetTSA.m');
path=path(1:(end-length('/Code/ProjetTSA.m')));

nomVideo=input('Quelle video voulez-vous etudier ?\nVoici le choix de video que vous avez : singleball.mp4, singleballhomemade.mp4\n\n', 's') ;

if strcmp(nomVideo, 'singleball.mp4')
    path=fullfile(path, 'Donnees', 'Video1');
    cd(path)
    delta_t=1.2; % periode d'echantillonage de la video
else
    if strcmp(nomVideo, 'singleballhomemade.mp4')
        path=fullfile(path, 'Donnees', 'Video2');
        cd(path)
        delta_t=1.2; % periode d'echantillonage de la video
    else
        disp('La video que vous avez entree n''existe pas');
        return
    end
end

informationSurDossier=dir(fullfile('VideoEnPNG', '*.png'));
NB_FRAMES=size(informationSurDossier,1);

%% Lecture de la video

%v=VideoReader(fullfile(path,nomVideo));

%% Visionnage de la video

%currAxes=axes;
%while hasFrame(v)
%    vidFrame=readFrame(v);
%    image(vidFrame, 'Parent', currAxes);
%    currAxes.Visible='off';
%    pause(1/v.FrameRate);
%end

%% Sauvegarde de chaque image de la video en png

%v.CurrentTime=0;
%nbFrame=0;
%while hasFrame(v)
%    vidFrame=readFrame(v);
%    nbFrame=nbFrame+1;
%    imwrite(vidFrame,fullfile('VideoEnPNG', [num2str(nbFrame) '.png']));
%end

%% Partie 1 : Traitement d'image
%% Detection de l'objet et stockage des centres de masse dans cell_position_objet

cell_position_objet=cell(1);
% cell_position_objet stockera :
% les coordonnees des centres de masse des objets detectes
% [] si aucun objet n'a ete detecte

i=1;
nbFrame=1;
tempImage=imread(fullfile('VideoEnPNG', [num2str(i), '.png'])); % lecture de la premiere image
empilementImagesBinaires=zeros(size(tempImage,1), size(tempImage,2)); % initialisation de empilementImagesBinaires

for i=1:NB_FRAMES
    image_tk=tempImage;

    if i<NB_FRAMES
        image_tkPlus1=imread(fullfile('VideoEnPNG', [num2str(i+1), '.png']));
        nbFrame=nbFrame+1;
        tempImage=image_tkPlus1;

        A=rgb2gray(image_tk); % A contient image_tk en noir et blanc
        B=rgb2gray(image_tkPlus1); % B contient image_tkPlus1 en noir et blanc
        C=imabsdiff(A, B); % C contient la valeur absolue de A-B
        thresh=graythresh(C); % calcul de l'intensite normalisee de l'image (entre 0 et 1)

        if (thresh>0.05) % au moins un objet a ete detecte (0.05 a ete choisi de maniere empirique)
            ib=imbinarize(C, thresh); % convertit l'image C en image binaire
            ib_sansbruit=bwareaopen(ib, 20, 8); % retrait des objets de moins de 20 pixels (bruit)

            s=regionprops(bwlabel(ib_sansbruit), 'centroid'); % determine entre autres le centre de masse de chaque objet
            c=[s.Centroid]; % centre(s) de masse

            cell_position_objet{nbFrame}=c; % stocke le/les centres de masse

            empilementImagesBinaires=empilementImagesBinaires+ib_sansbruit; % empilement des images binaires

        else % aucun objet n'a ete detecte
            cell_position_objet{nbFrame}=[]; % stocke le fait qu'aucun objet n'a ete detecte
        end
    end
end

%% Moyennage des centres de masse

cell_pos_bis=[];
% cell_pos_bis stockera :
% la moyenne de deux centres de masse cell_position_objet
% un centre de masse de cell_position_objet
% [] si aucun objet n'a ete detecte

i=1;
k=1;
estVraiment2_4=true;
while i<=(length(cell_position_objet)-1)

    if isempty(cell_position_objet{i})
        cell_pos_bis{k}=[];
        estVraiment2_4=true;
    else if ((length(cell_position_objet{i})==2) && ((length(cell_position_objet{i+1})==4)))
            if (estVraiment2_4==true)
                cell_pos_bis{k}=[];
                k=k+1;
            end
            cell_pos_bis{k}=mean([cell_position_objet{i}(1:2); cell_position_objet{i+1}(1:2)]);
            cell_position_objet{i+1}(1:2)=[];
            estVraiment2_4=false;
        else if ((length(cell_position_objet{i})==4) && ((length(cell_position_objet{i+1})==4)))
                cell_pos_bis{k}=cell_position_objet{i}(1:2);
                k=k+1;
                cell_pos_bis{k}=mean([cell_position_objet{i}(3:4); cell_position_objet{i+1}(1:2)]);
                cell_position_objet{i+1}(1:2)=[];
                estVraiment2_4=false;
            else if ((length(cell_position_objet{i})==4) && ((length(cell_position_objet{i+1})==2)))
                    cell_pos_bis{k}=cell_position_objet{i}(1:2);
                    k=k+1;
                    cell_pos_bis{k}=mean([cell_position_objet{i}(3:4); cell_position_objet{i+1}(1:2)]);
                    cell_position_objet(i+1)=[];
                 else if ((length(cell_position_objet{i})==2) && ((length(cell_position_objet{i+1})==2)))
                        cell_pos_bis{k}=cell_position_objet{i};
                        else if ((length(cell_position_objet{i})==2) && (isempty(cell_position_objet{i+1})))
                            cell_pos_bis{k}=cell_position_objet{i};
                                else if ((length(cell_position_objet{i})==4) && (isempty(cell_position_objet{i+1})))
                                    cell_pos_bis{k}=cell_position_objet{i}(1:2);
                                    k=k+1;
                                    cell_pos_bis{k}=cell_position_objet{i}(3:4);
                                    i=i+1;
                                    end
                            end
                     end
                end
            end
        end
    end
    k=k+1;
    i=i+1;
end

% Dans le cas ou l'objet apparait sur l'image au dernier frame
if ~isempty(cell_position_objet{i})
    cell_pos_bis{k}=cell_position_objet{i}(1:2);
end

%% Retrait des positions quand l'objet n'est plus detecte en fin de video

% Retrait des [] a la fin de cell_pos_bis
% car cela veut dire que l'objet n'est plus detecte dans la video
j=length(cell_pos_bis);
while isempty(cell_pos_bis{j})
    if isempty(cell_pos_bis{j})
        cell_pos_bis(j)=[];
    end
    j=length(cell_pos_bis);
end

%% Moyenne des points qui sont encore trop pres (en fait ce sont les memes points)

i=1;
while (i<=length(cell_pos_bis)-1)
    if (~isempty(cell_pos_bis{i}) && ~isempty(cell_pos_bis{i+1}))
        if ((abs(cell_pos_bis{i}(1)-cell_pos_bis{i+1}(1))<=3) && (abs(cell_pos_bis{i}(2)-cell_pos_bis{i+1}(2))<=3))
            cell_pos_bis{i}=mean([cell_pos_bis{i}; cell_pos_bis{i+1}]);
            cell_pos_bis(i+1)=[];
        end
    end
    i=i+1;
end

%% Affichage des detections des centres de masse de l'objet

% cell_pos_bis en matrice pour faciliter l'affichage
matrice_pos_bis=[];

for i=1:length(cell_pos_bis)
    if ~isempty(cell_pos_bis{i})
        matrice_pos_bis=[matrice_pos_bis; cell_pos_bis{i}(1), cell_pos_bis{i}(2)];
    end
end

figure
subplot(2,1,1)
imshow(empilementImagesBinaires)
hold on
    plot(matrice_pos_bis(:,1), matrice_pos_bis(:,2), 'r+')
    title('Image binaire');
    xlabel('Axe x');
    ylabel('Axe y');
    legend('Positions detectees avec traitement d''image')
hold off

subplot(2,1,2)
imshow(A)
hold on
    plot(matrice_pos_bis(:,1), matrice_pos_bis(:,2), 'r+')
    title('Image de depart en noir et blanc');
    xlabel('Axe x');
    ylabel('Axe y');
    legend('Positions detectees avec traitement d''image')
hold off

%% Partie 2
%% Filtre de Kalman

clearvars -except cell_pos_bis empilementImagesBinaires delta_t matrice_pos_bis A
clear trackingObjet
clear kalmanFilter
clear kalmanFilterAjustementsVitesseAcceleration

Matrice_y=trackingObjet(cell_pos_bis, delta_t);

%% Affichages des resultats

A=imread(fullfile('VideoEnPNG', [num2str(1), '.png'])); % 1.png (souvent arriere plan)

%%

figure
hold on
    grid on
    plot(Matrice_y(:,1), Matrice_y(:,2), 'gO');
    plot(matrice_pos_bis(:,1), matrice_pos_bis(:,2), 'b+')
    set(gca,'Ydir','reverse')
    title('Positions successives de l''objet');
    xlabel('Axe x');
    ylabel('Axe y (inverse)');
    legend('Positions tracking objet (filtre de Kalman)',...
        'Positions detectees avec traitement d''image')
hold off

%%

figure
hold on
subplot(2,1,1)
imshow(empilementImagesBinaires)
hold on
    plot(Matrice_y(:,1), Matrice_y(:,2), 'gO-')
    plot(matrice_pos_bis(:,1), matrice_pos_bis(:,2), 'r+')
    title('Image binaire');
    xlabel('Axe x');
    ylabel('Axe y');
    legend('Positions tracking objet (filtre de Kalman)',...
        'Positions detectees avec traitement d''image')
hold off

subplot(2,1,2)
imshow(A)
hold on
    plot(Matrice_y(:,1), Matrice_y(:,2), 'gO-')
    plot(matrice_pos_bis(:,1), matrice_pos_bis(:,2), 'r+')
    title('Image de depart en couleur');
    xlabel('Axe x');
    ylabel('Axe y');
    legend('Positions tracking objet (filtre de Kalman)',...
        'Positions detectees avec traitement d''image')
hold off
hold off

%%

figure
hold on
image(A)
hold on
    plot(Matrice_y(:,1), Matrice_y(:,2), 'gO-')
    plot(matrice_pos_bis(:,1), matrice_pos_bis(:,2), 'r+')
    set(gca,'Ydir','reverse')
    ax = gca;
    ax.Visible = 'off';
    title('Image de depart en couleur');
    xlabel('Axe x');
    ylabel('Axe y');
    legend('Positions tracking objet (filtre de Kalman)',...
        'Positions detectees avec traitement d''image')
hold off
hold off