# Quadruped

Attention: pour le mode goto, le robot va dans la direction de la cible mais le point d'arrivée n'est pas précis.
           Pour le mode walk, tourner avec t_speed interrompt la marche et reprend que si t_speed est remis à 0

Dépendances à installer:

    pip install numpy scipy pybullet jupyter matplotlib

Ensuite:

    python quadruped.py -m mode [-x x-cible -y y-cible -t t-cible]
    
Explication du nombres de solutions possibles:
- 0 solutions: l'ensemble des points inateignables en dehors du quart de sphère
- 1 solution: cas où la patte est tendue ce qui correpond au quart de sphère
- 2 solutions: il y a une symétrie 
- infinité: l'ensemble des points du premier moteur

Pour régler ces problèmes, j'ai créé deux demi-sphère dans le repère de 
la patte. Le premier demi-cercle est de rayon L2+L3+L4 et correspond donc
à la portée maximale d'une patte. Puis pour éviter l'infinité de possibilités
du premier moteurs et éviter les collision, j'ai créé une deuxième demi-sphère de rayon 
supérieur à L1. Tous les positions dans cette demi-sphère sont interdits.
De plus j'ai encadré mes différents 'cos' en ramenant les valeurs à -1 si 
elles étaient inférieur à -1 et à 1 si elles étaient supérieur à 1.
Ainsi je forme comme une sorte d'essui glace dans le repère de ma patte.
