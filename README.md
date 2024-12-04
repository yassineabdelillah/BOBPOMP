# BOBPOMP
# Simulator BOB robots POMPIER version
# CLASSFILE PYTHON 

import numpy as np



class simulateur:
    def __init__(self):
        vitesseSimu = 1
        self.vitesseSimu = vitesseSimu
        self.listeFeu = []
        self.listeRobot = []

    def lancerSimulation(self):
        print("La simulation est lancee")

    def ajouterFeu(self):
        self.listeFeu=self.listeFeu.append(('F',feu.coordonnerFeu))
        print("Un feu est ajoute")

    def ajouterRobot(self):
        self.listeRobot=self.listeRobot.append(('R',robot.id, robot.coordonnerRobot))
        print("Un robot est ajoute")
    
    def lireCarte(self):
        print("La carte est lue")
    
    def arreterSimulation(self):
        print("La simulation est arretee")
    
    def pauseSimulation(self):
        print("La simulation est en pause")
    
    def reprendreSimulation(self):
        print("La simulation est reprise")
    
    def getVitesseSimu(self):
        return self.vitesseSimu
    def setVitesseSimu(self, vitesseSimu):
        self.vitesseSimu = vitesseSimu
    




class carte:
    def __init__(self):
        self.carte =np.zeros(2000,2000)
        self.coordonneerBase=self.carte[0,0]
    def afficherCarte(self):
        print(self.carte)






class feu:
    def __init__(self):
        self.nbFeu = 0
        self.numFeu= 0
        self.coordonneerFeu = [0,0]
        self.ampleur = 20

    def seDeclencher(self):
        carte.carte(self.coordonneerFeu)== 'Feu'
        print("Un feu s'est declenche")






class robot:
    def __init__(self):
        self.coordonneerRobot = [carte.coordonnerBase[0],carte.coordonnerBase[1]]
        self.vitesse = 0
        self.idRobot = 0
        self.etat = "a la base"
        self.stockFeu = 2
        self.robotDispo = True
        self.typeRobot = ""
        self.constante = 1
        self.distance=0 
        self.constante=1       
        
    def eteindreFeu(self) :
        if feu.coordonneerFeu== self.coordonnerRobot:
            self.stockFeu=self.stockFeu-1
            T=('F',feu.coordonnerFeu)
            simulateur.listeFeu= simulateur.listeFeu.remove(T)
            print("Le feu est éteint")
        else :
            a=np.rand(0,1)
            if a==0 :
                ampleurFeu=ampleurFeu-1
            else :
                ampleurFeu=ampleurFeu+1
        
    def seDeplacer(self):
        if (feu.coordonneerFeu(0)-self.coordonnerRobot(0))<0 and (feu.coordonneerFeu(1)-self.coordonnerRobot(1))<0 and len(feu.listeFeu)>0:
            self.coordonneerRobot[0]=self.coordonneerRobot[0]-self.constante
            self.coordonneerRobot[1]=self.coordonneerRobot[1]-self.constante
            self.etat == "en déplacement"
            carte.carte(self.coordonneerRobot[0],self.coordonneerRobot[1])=='R'
        elif (feu.coordonneerFeu(0)-self.coordonnerRobot(0))>0 and (feu.coordonneerFeu(1)-self.coordonnerRobot(1))>0 and len(feu.listeFeu)>0:
            self.coordonneerRobot[0]=self.coordonneerRobot[0]+self.constante
            self.coordonneerRobot[1]=self.coordonneerRobot[1]+self.constante
            self.etat == "en déplacement"
            carte.carte(self.coordonneerRobot[0],self.coordonneerRobot[1])=='R'
        elif (feu.coordonneerFeu(0)-self.coordonnerRobot(0))<0 and (feu.coordonneerFeu(1)-self.coordonnerRobot(1))>0 and len(feu.listeFeu)>0:   
            self.coordonneerRobot[0]=self.coordonneerRobot[0]-self.constante
            self.coordonneerRobot[1]=self.coordonneerRobot[1]+self.constante
            self.etat == "en déplacement"
            carte.carte(self.coordonneerRobot[0],self.coordonneerRobot[1])=='R'
        elif (feu.coordonneerFeu(0)-self.coordonnerRobot(0))>0 and (feu.coordonneerFeu(1)-self.coordonnerRobot(1))<0 and len(feu.listeFeu)>0:
            self.coordonneerRobot[0]=self.coordonneerRobot[0]+self.constante
            self.coordonneerRobot[1]=self.coordonneerRobot[1]-self.constante
            self.etat == "en déplacement"
            carte.carte(self.coordonneerRobot[0],self.coordonneerRobot[1])=='R'

    def calculerDistance(self):
        self.distance = np.sqrt((feu.coordonneerFeu[0]-self.coordonneerRobot[0])**2+(feu.coordonneerFeu[1]-self.coordonneerRobot[1])**2)
        print("Le robot calcule la distance")

    def remplirReservoir(self):
        if self.stockFeu==0:
            self.seDeplacer(self, carte.coordonneerBase)
            self.stockFeu=2
        print("Le robot se rempli")
        
    def machine_a_etats(self, feu_coordonnees=None, ampleurFeu=None, nouvelle_mission=None):
            if self.etat == "a la base":
                print("État : À la base")
                if self.stockFeu < 2:
                    self.remplirReservoir()  # Le remplissage est géré ici
                if feu_coordonnees is not None:  # Une mission est disponible
                    self.calculerDistance(feu_coordonnees)
                    self.etat = "en attente de mission"

            elif self.etat == "en attente de mission":
                print("État : En attente de mission")
                if feu_coordonnees is not None:  # Une mission est confirmée
                    print("Nouvelle mission reçue !")
                    self.calculerDistance(feu_coordonnees)
                    self.etat = "en déplacement"

            elif self.etat == "en déplacement":
                print("État : En déplacement")
                if feu_coordonnees is not None:  # Se déplacer vers le feu
                    self.seDeplacer(feu_coordonnees)
                    if self.coordonneerRobot == feu_coordonnees:  # Vérifier si arrivé à destination
                        print("Robot arrivé à destination.")
                        self.etat = "extinction du feu"

            elif self.etat == "extinction du feu":
                print("État : Extinction du feu")
                if feu_coordonnees is not None and ampleurFeu is not None:  # Si à proximité du feu
                    self.eteindreFeu(feu_coordonnees, ampleurFeu)  # Éteindre le feu
                    if self.stockFeu == 0:  # Si le réservoir est vide
                        self.etat = "retour à la base"
                    else:  # Si encore du stock, retourner à l'état initial
                        self.etat = "a la base"

            elif self.etat == "retour à la base":
                print("État : Retour à la base")
                self.seDeplacer([0, 0])  # Retourner aux coordonnées de la base
                if nouvelle_mission and self.stockFeu >= 1:  # Nouvelle mission en chemin
                    print("Nouvelle mission reçue pendant le retour à la base !")
                    feu_coordonnees = nouvelle_mission  # Mise à jour des coordonnées pour la nouvelle mission
                    self.calculerDistance(feu_coordonnees)
                    self.etat = "en déplacement"
                elif self.coordonneerRobot == [0, 0]:  # Si arrivé à la base
                    print("Robot est arrivé à la base.")
                    self.etat = "a la base"  # Passer à l'état "a la base"







class manager:
    def __init__(self):
      feu.nbFeu=len(simulateur.listeFeu)

def affecter_robot_aux_feu(simulateur.listeRobot, simulateur.listeFeu):
    # Function to assign robots to fires based on the number of robots and fires
    def calculer_distance(coordonneer1, coordonneer2):
        return np.sqrt((coordonneer2[0] - coordonneer1[0]) ** 2 + (coordonneer2[1] - coordonneer1[1]) ** 2)

    def affecter_un_robot_a_un_feu(robot, feu):
        """Assign a robot to a fire"""
        robot['assigned'] = True
        print(f"Robot {robot['coordonneerRobot']} affecté au feu {feu['coordonneerFeu']}")

    def trouver_feu_le_plus_proche(robot, simulateur.listeFeu):
        min_distance = float('inf')
        nearest_feu = None
        for feu in simulateur.listeFeu:
            distance = calculer_distance(robot['coordonneerRobot'], feu['coordonneerFeu'])
            if distance < min_distance:
                min_distance = distance
                nearest_feu = feu
        return nearest_feu

    def trouver_robot_le_plus_proche(feu, simulateur.listeRobot):
        min_distance = float('inf')
        nearest_robot = None
        for robot in simulateur.listeRobot:
            if not robot['assigned']:  # Only consider unassigned robots
                distance = calculer_distance(robot['coordonneerRobot'], feu['coordonneerFeu'])
                if distance < min_distance:
                    min_distance = distance
                    nearest_robot = robot
        return nearest_robot
    num_robots = len(simulateur.listeRobot)
    num_feux = len(simulateur.listeFeu)

    # Cas 1: Equal number of robots and fires
    if num_robots == num_feux:
        for robot, feu in zip(simulateur.listeRobot, simulateur.listeFeu):
            affecter_un_robot_a_un_feu(robot, feu)
    
    # Cas 2: More fires than robots
    elif num_feux > num_robots:
        for i in range(num_robots):
            nearest_feu = trouver_feu_le_plus_proche(simulateur.listeRobot[i], simulateur.listeFeu)
            affecter_un_robot_a_un_feu(simulateur.listeRobot[i], nearest_feu)

    # Cas 3: More robots than fires
    else:  # More robots than fires
        for feu in simulateur.listeFeu:
            nearest_robot = trouver_robot_le_plus_proche(feu, simulateur.listeRobot)
            affecter_un_robot_a_un_feu(nearest_robot, feu)
                
        # Assign remaining robots to the nearest fires
        remaining_robots = [robot for robot in simulateur.listeRobot if not robot['assigned']]
        for robot in remaining_robots:
            nearest_feu = trouver_feu_le_plus_proche(robot, simulateur.listeFeu)
            affecter_un_robot_a_un_feu(robot, nearest_feu)
    print("L'affectation des robots aux feux est terminée.")
    
    def calculerDistancebymanager(self):
        LISTE=[]
        MATRICEDISTANCE=np.zeros(len(simulateur.listeFeu),len(simulateur.listeRobot))
        for i in range(len(simulateur.listeFeu)):
            for j in range(len(simulateur.listeRobot)):
                TF=simulateur.listeFeu[i]
                TR=simulateur.listeRobot[j]
                CoorF=TF[1]
                CoorR=TR[2]
                Dij=np.sqrt((CoorF[0]-CoorR[0])**2+(CoorF[1]-CoorR[1])**2)
                MATRICEDISTANCE[i][j]= Dij
                LISTE=LISTE.append(Dij)
        return MATRICEDISTANCE
