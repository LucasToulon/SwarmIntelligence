import java.util.ArrayList;


public	class Vehicle{
    static int allId = 0;
    int id;					//Fahrzeug-ID
    double rad_sep;			//Radius f�r Zusammenbleiben
    double rad_zus;			//Radius f�r Separieren
    double rad_fol;			//Radius f�r Folgen
    double rad_sepSchafzuSchaeferhund; //Radius für Erkennung als Schaf
    double rad_ErkennungSchaeferhundzuSchaf; //Radius für Erkennung als Schäferhund
    int type;				//Fahrzeug-Type (0: Verfolger; 1: Anf�hrer)
    final double FZL;		//L�nge
    final double FZB;		//Breite
    final double RESETrad_ErkennungSchaeferhundzuSchaf;
    final double INCREMENTrad_ErkennungSchaeferhundzuSchaf;

    //Aktuelle Bewegung 	(wird in der Methode bewegen() berechnet)
    double[] pos;			//Position
    double[] vel;			//Geschwindigkeit
    double[] acc;			//Beschleunigung

    //Steuerungsparameter 	(wird in der Methode steuerparameter_festlegen() berechnet)
    double[] acc_dest;		//Zielbeschleunigung
    final double max_acc;	//Maximale Beschleunigung
    final double max_accSchaeferhund; //Schäferhund darf anders gesteuert werden
    final double max_vel;	//Maximale Geschwindigkeit
    final double max_velSchaeferhund; //Schäferhund darf anders gesteuert werden
    final double max_cor; //Korrekturfaktor für die Richtung des Tores

    //Zuk�nftige Bewegung 	(wird in der Methode steuern() berechnet)
    double[] pos_new;		//Neue Position
    double[] vel_new;		//Neue Geschwindigkeit
    double[] acc_new;   	//Neue Beschleunigung

    Vehicle(){
        allId++;
        this.id       		= allId;
        this.FZL          	= 2;
        this.FZB          	= 1;
        this.rad_sep        = 2;//50
        this.rad_zus        = 2;//25
        this.rad_fol        = 200;
        this.rad_sepSchafzuSchaeferhund = 40;
        this.rad_ErkennungSchaeferhundzuSchaf = 25;
        this.RESETrad_ErkennungSchaeferhundzuSchaf = 25;
        this.INCREMENTrad_ErkennungSchaeferhundzuSchaf = 5;
        this.type           = 0;
        this.max_acc        = 0.5;//0.1
        this.max_accSchaeferhund= 0.3;
        this.max_vel        = 0.08;
        this.max_velSchaeferhund      = 0.2;
        this.max_cor        = 0.01;

        pos		 			= new double[2];
        vel    	 			= new double[2];
        acc		 			= new double[2];
        acc_dest			= new double[2];
        pos_new	 			= new double[2];
        vel_new	 			= new double[2];
        acc_new	 			= new double[2];

        pos[0]              = Simulation.pix*500*Math.random();
        pos[1]              = Simulation.pix*500*Math.random();
        vel[0]     			= max_vel*Math.random();
        vel[1] 	    		= max_vel*Math.random();
        acc[0]     			= max_vel*Math.random();
        acc[1] 	    		= max_vel*Math.random();
        acc_dest[0]         = 0;
        acc_dest[1]         = 0;
        vel_new[0] 			= vel[0];
        vel_new[1] 			= vel[1];
        pos_new[0]          = pos[0];
        pos_new[1]          = pos[1];
        acc_new[0]          = acc[0];
        acc_new[1]          = acc[1];
    }

    //cohesion
    double[] zusammenbleiben(ArrayList<Vehicle> all){
        ArrayList<Vehicle>  neighbours = new ArrayList<Vehicle>();
        double[] pos_dest   = new double[2];
        double[] vel_dest   = new double[2];
        double[] acc_dest   = new double[2];
        acc_dest[0]         = 0;
        acc_dest[1]         = 0;
        for(int i=0;i<all.size();i++){
            Vehicle v = all.get(i);
            if(v.id != this.id && v.type!=1){
                double dist = Math.sqrt(Math.pow(v.pos[0]-this.pos[0],2) + Math.pow(v.pos[1]-this.pos[1],2));
                if(dist < rad_zus){
                    neighbours.add(v);
                }
            }
        }
        if(neighbours.size() > 0){
            //1. pos_dest
            pos_dest[0]     = 0;
            pos_dest[1]     = 0;
            for(int i=0;i<neighbours.size();i++){
                Vehicle v   = neighbours.get(i);
                pos_dest[0] = pos_dest[0] + v.pos[0];
                pos_dest[1] = pos_dest[1] + v.pos[1];
            }
            pos_dest[0] = pos_dest[0] / neighbours.size();
            pos_dest[1] = pos_dest[1] / neighbours.size();

            //2. vel_dest
            vel_dest[0]  = pos_dest[0]-pos[0];
            vel_dest[1]  = pos_dest[1]-pos[1];

            //3. maximum speed
            vel_dest     = normalize(vel_dest);//
            vel_dest[0]  = vel_dest[0]*max_vel;//
            vel_dest[1]  = vel_dest[1]*max_vel;//

            //4. acc_dest
            acc_dest[0]  = vel_dest[0]-vel[0];
            acc_dest[1]  = vel_dest[1]-vel[1];

        }
        return acc_dest;
    }

    double[] separierenvomSchaeferhund(ArrayList<Vehicle> all){

        ArrayList<Vehicle> myneighbours = new ArrayList<Vehicle>();
        double[] vel_dest = new double[2];
        double[] acc_dest = new double[2];
        acc_dest[0]       = 0;
        acc_dest[1]       = 0;
        //Wenn schon drüber, keine Korrektur mehr
        if((pos[0] >= 510*Simulation.pix)){
            return acc_dest;
        }
        for(int i=0;i<all.size();i++){
            Vehicle v = all.get(i);
            if(v.type==1){//nur Separation vom Anführer
                double dist = Math.sqrt(Math.pow(v.pos[0]-this.pos[0],2) + Math.pow(v.pos[1]-this.pos[1],2));
                if(dist < rad_sepSchafzuSchaeferhund){
                    myneighbours.add(v);
                }
            }
        }
        if(myneighbours.size() > 0){
            //1. Zielrichtung
            vel_dest[0] = 0;
            vel_dest[1] = 0;
            for(int i=0;i<myneighbours.size();i++){
                Vehicle v = myneighbours.get(i);
                double[] tmp = new double[2];
                double dist;

                tmp[0]  = v.pos[0] - pos[0];
                tmp[1]  = v.pos[1] - pos[1];
                dist    = rad_sepSchafzuSchaeferhund-length(tmp);
                //dist = length(tmp);
                if(dist < 0) System.out.println("fehler in rad2");
                tmp          = normalize(tmp);
                tmp[0]       = -tmp[0] * dist;
                tmp[1]       = -tmp[1] * dist;
                vel_dest[0]  = vel_dest[0] + tmp[0];
                vel_dest[1]  = vel_dest[1] + tmp[1];
            }
            //2. Zielgeschwindigkeit
            vel_dest     = normalize(vel_dest);
            vel_dest[0]  = vel_dest[0]*max_vel;
            vel_dest[1]  = vel_dest[1]*max_vel;

            //3. Korrektur der Zielgeschwindigkeit mit Richtung "zum Tor"
            double [] correction = new double[2];
            //"irgendwo beim Tor" damit nicht alle zu einem fixen Punkt laufen
            int randomY = 355 + (int)(Math.random() * ((445 - 355) + 1));
            correction[0] = Simulation.GOAL[0]*Simulation.pix - pos[0] + 20;
            correction[1] = randomY*Simulation.pix - pos[1];

            //Das soll nur "in die Richtung" gehen
            correction[0]  = correction[0]*max_cor;
            correction[1]  = correction[1]*max_cor;

            vel_dest[0]  = vel_dest[0]+correction[0];
            vel_dest[1]  = vel_dest[1]+correction[1];

            //4. Zielbeschleunigung
            acc_dest[0]  = vel_dest[0]-vel[0];
            acc_dest[1]  = vel_dest[1]-vel[1];
        }
        return acc_dest;
    }

    double[] separieren(ArrayList<Vehicle> all){
        ArrayList<Vehicle> myneighbours = new ArrayList<Vehicle>();
        double[] vel_dest = new double[2];
        double[] acc_dest = new double[2];
        acc_dest[0]       = 0;
        acc_dest[1]       = 0;

        for(int i=0;i<all.size();i++){
            Vehicle v = all.get(i);
            if(v.id != this.id && v.type!=1){//keine Separation vom Anführer
                double dist = Math.sqrt(Math.pow(v.pos[0]-this.pos[0],2) + Math.pow(v.pos[1]-this.pos[1],2));
                if(dist < rad_sep){
                    myneighbours.add(v);
                }
            }
        }
        if(myneighbours.size() > 0){
            //1. Zielrichtung
            vel_dest[0] = 0;
            vel_dest[1] = 0;
            for(int i=0;i<myneighbours.size();i++){
                Vehicle v    = myneighbours.get(i);
                double[] tmp = new double[2];
                double dist;

                tmp[0]  = v.pos[0] - pos[0];
                tmp[1]  = v.pos[1] - pos[1];
                dist    = rad_sep-length(tmp);
                //dist = length(tmp);
                if(dist < 0) System.out.println("fehler in rad");
                tmp          = normalize(tmp);
                tmp[0]       = -tmp[0] * dist;
                tmp[1]       = -tmp[1] * dist;
                vel_dest[0]  = vel_dest[0] + tmp[0];
                vel_dest[1]  = vel_dest[1] + tmp[1];

            }

            //2. Zielgeschwindigkeit
            vel_dest     = normalize(vel_dest);
            vel_dest[0]  = vel_dest[0]*max_vel;
            vel_dest[1]  = vel_dest[1]*max_vel;

            //3. Zielbeschleunigung
            acc_dest[0]  = vel_dest[0]-vel[0];
            acc_dest[1]  = vel_dest[1]-vel[1];
        }

        return acc_dest;
    }

    double[] ausrichten(ArrayList<Vehicle> all){
        ArrayList<Vehicle>  neighbours = new ArrayList<Vehicle>();
        double[] vel_dest   = new double[2];
        double[] acc_dest   = new double[2];
        acc_dest[0]         = 0;
        acc_dest[1]         = 0;

        for(int i=0;i<all.size();i++){
            Vehicle v = all.get(i);
            if(v.id != this.id && v.type!=1){
                double dist = Math.sqrt(Math.pow(v.pos[0]-this.pos[0],2) + Math.pow(v.pos[1]-this.pos[1],2));
                if(dist < rad_zus){
                    neighbours.add(v);
                }
            }
        }

        if(neighbours.size() > 0){
            //1. Zielrichtung
            vel_dest[0]       = 0;
            vel_dest[1]       = 0;
            for(int i=0;i<neighbours.size();i++){
                Vehicle v = neighbours.get(i);
                vel_dest[0]   = vel_dest[0] + v.vel[0];
                vel_dest[1]   = vel_dest[1] + v.vel[1];
            }

            //2. Zielgeschwindigkeit
            vel_dest    = normalize(vel_dest);
            vel_dest[0] = vel_dest[0]*max_vel;
            vel_dest[1] = vel_dest[1]*max_vel;

            //3. Zielbeschleunigung
            acc_dest[0] = vel_dest[0]-vel[0];
            acc_dest[1] = vel_dest[1]-vel[1];
        }

        return acc_dest;
    }

    double[] driveAndCollect(ArrayList<Vehicle> all){
        //In der Umgebung schauen, welche Schafe da sind
        ArrayList<Vehicle> schaeferHundNeighbourHood = new ArrayList<Vehicle>();
        double[] pos_dest   = new double[2];
        double[] acc_dest   = new double[2];
        acc_dest[0]         = 0;
        acc_dest[1]         = 0;
        pos_dest[0]     = 0;
        pos_dest[1]     = 0;
        boolean search = true;
        //Wir wollen immer mind. 1 Schaf in der Umgebung
        while(search){
            for(int i=0;i<all.size();i++){
                Vehicle v = all.get(i);
                //Nur die, die noch links vom Zaun sind
                if(v.id != this.id && v.type!=1 && v.pos[0] < 500*Simulation.pix){
                    double dist = Math.sqrt(Math.pow(v.pos[0]-this.pos[0],2) + Math.pow(v.pos[1]-this.pos[1],2));
                    if(dist < rad_ErkennungSchaeferhundzuSchaf){
                        schaeferHundNeighbourHood.add(v);
                        pos_dest[0] = pos_dest[0] + v.pos[0];
                        pos_dest[1] = pos_dest[1] + v.pos[1];
                    }
                }
            }
            //solange kein Schaf in der Umgebung ist, den Radius vergrößern und nochmal probieren, ansonsten für das nächste Mal wieder resetten
            if(schaeferHundNeighbourHood.size() == 0){
                //Radius vergrößern
                rad_ErkennungSchaeferhundzuSchaf = rad_ErkennungSchaeferhundzuSchaf + INCREMENTrad_ErkennungSchaeferhundzuSchaf;
            }else{
                //Radius wieder resetten
                search = false;
                rad_ErkennungSchaeferhundzuSchaf = RESETrad_ErkennungSchaeferhundzuSchaf;
            }
        }
        //faktisch wird es immer größer als 0 sein aber sicher ist sicher
        if(schaeferHundNeighbourHood.size() > 0){
            //1. den Mittelpunkt ermitteln.
            pos_dest[0] = pos_dest[0] / schaeferHundNeighbourHood.size();
            pos_dest[1] = pos_dest[1] / schaeferHundNeighbourHood.size();

            //pos_dest = Mitte der Schafe
            //GOAL
            //gm = pos_dest - GOAL
            double[] vectorGoalMitte = new double[2];
            vectorGoalMitte[0]=pos_dest[0]-Simulation.GOAL[0]*Simulation.pix;
            vectorGoalMitte[1]=pos_dest[1]-Simulation.GOAL[1]*Simulation.pix;
            double [] destination = new double[2];
            destination[0] = Simulation.GOAL[0]*Simulation.pix + Simulation.OVERSHOT*vectorGoalMitte[0];
            destination[1] = Simulation.GOAL[1]*Simulation.pix + Simulation.OVERSHOT*vectorGoalMitte[1];
            //Sichergehen, dass wir im Rahmen bleiben
            if(destination[0]<0) destination[0]=0;
            if(destination[0]>499*Simulation.pix) destination[0]=499*Simulation.pix;
            if(destination[1]<0) destination[0]=0;
            if(destination[1]>600*Simulation.pix) destination[0]=600*Simulation.pix;

            //Beschleunigung setzen
            acc_dest[0]  = destination[0]-pos[0];
            acc_dest[1]  = destination[1]-pos[1];

        }

        if(Math.random()<0.01){
            acc_dest[0]  = max_accSchaeferhund*Math.random();
            acc_dest[1]  = max_accSchaeferhund*Math.random();
        }

        return acc_dest;
    }


    public void steuerparameter_festlegen(ArrayList<Vehicle> allVehicles){

        double[] acc_dest1 = new double[2];
        double[] acc_dest2 = new double[2];
        double[] acc_dest3 = new double[2];
        double[] acc_dest4 = new double[2];
        double f1  = 0.01; //0.05
        double f2  = 0.1; //0.55
        double f3  = 0.01; //0.4
        double f4  = 0.9; //0.9

        if(type == 1){
            //Der Schäferhund kriegt eine ganz andere Beschleunigung
            this.acc_dest = driveAndCollect(allVehicles);
        } else{
            //acc_dest1   = zusammenbleiben(allVehicles);
            //acc_dest1   = folgen(allVehicles);
            //Die Schafe sollen nicht unbedingt zusammenbleiben
            acc_dest2   = separieren(allVehicles);
            acc_dest3   = ausrichten(allVehicles);
            acc_dest4   = separierenvomSchaeferhund(allVehicles);

            this.acc_dest[0] = (/*(f1 * acc_dest1[0])+*/ (f2 * acc_dest2[0]) + (f3 * acc_dest3[0]) + (f4 * acc_dest4[0]));
            this.acc_dest[1] = (/*(f1 * acc_dest1[1])+*/ (f2 * acc_dest2[1]) + (f3 * acc_dest3[1]) + (f4 * acc_dest4[1]));

        }


    }

    void steuern(){

        //1. Beschleunigung berechnen
        acc_new = truncate(acc_dest, max_acc);

        //2. Neue Geschwindigkeit berechnen
        vel_new[0] = vel[0]+acc_new[0];
        vel_new[1] = vel[1]+acc_new[1];
        if(this.type == 1){
            //max_velHaifisch
            vel_new = truncate(vel_new, max_velSchaeferhund);
        }else{
            vel_new  = truncate(vel_new, max_vel);
        }

        //3. Neue Position berechnen
        pos_new[0] = pos[0] + vel_new[0];
        pos_new[1] = pos[1] + vel_new[1];

        //4. Position ggf. korrigieren, falls Rand der Bewegungsfl�che erreicht
        if(pos_new[0] < 10*Simulation.pix){
            vel_new[0] = Math.abs(vel_new[0]);
            pos_new[0] = pos[0] + vel_new[0];
        }
        if(pos_new[0] > 990*Simulation.pix){
            vel_new[0] = -Math.abs(vel_new[0]);
            pos_new[0] = pos[0] + vel_new[0];
        }
        if(pos_new[1] < 10*Simulation.pix){
            vel_new[1] = Math.abs(vel_new[1]);
            pos_new[1] = pos[1] + vel_new[1];
        }
        if(pos_new[1] > 750*Simulation.pix){
            vel_new[1] = -Math.abs(vel_new[1]);
            pos_new[1] = pos[1] + vel_new[1];
        }

        //5. Position ggf. korrigieren, falls Zaun erreicht wird
        if(this.type == 0 && (pos_new[0] >= 499*Simulation.pix) && (pos_new[0] <= 501*Simulation.pix) && (pos_new[1] <350*Simulation.pix || pos_new[1]>450*Simulation.pix)){
            vel_new[0] = -(vel_new[0]);
            pos_new[0] = pos[0] + vel_new[0];
        }
        //6. Schafe bleiben rechts
        if(this.type == 0 && (pos_new[0] <= 501*Simulation.pix) && (pos[0] > 501*Simulation.pix)){
            vel_new[0] = -(vel_new[0]);
            pos_new[0] = pos[0] + vel_new[0];
        }
        //der Hund bleibt links
        if(this.type == 1 && pos_new[0] > 499*Simulation.pix && pos_new[0] < 501*Simulation.pix){
            vel_new[0] = -vel_new[0];
            pos_new[0] = pos[0] + vel_new[0];
        }
    }

    void bewegen(){
        pos[0] = pos_new[0];
        pos[1] = pos_new[1];
        vel[0] = vel_new[0];
        vel[1] = vel_new[1];
        acc[0] = acc_new[0];
        acc[1] = acc_new[1];
    }

    static double truncate(double x, double y){
        if(y < 0)System.out.println("Fehler truncate");
        if(x > 0)return Math.min(x,  y);
        else     return Math.max(x, -y);
    }


    static double[] normalize(double[] x){
        double[] res = new double[2];
        double  norm = Math.sqrt(Math.pow(x[0], 2)+Math.pow(x[1], 2));
        res[0]       = x[0];
        res[1]       = x[1];
        if(norm!=0){
            res[0] = x[0]/norm;
            res[1] = x[1]/norm;
        }

        return res;
    }

    static double[] truncate(double[] x, double y){
        if(y < 0)System.out.println("Fehler truncate");
        double[] res  = normalize(x);
        res[0]        = res[0]*truncate(length(x), y);
        res[1]        = res[1]*truncate(length(x), y);
        return res;
    }

    static double length (double[] x){
        double res = Math.sqrt(Math.pow(x[0], 2)+Math.pow(x[1], 2));
        return res;
    }

    static double winkel(double[] v1){
        //Winkel von v1 gegen�ber Koordinaten-X-Achse [0, 360[ gegen den Uhrzeigersinn

        double[] k = new double[2];
        double w;

        k[0] = 1;
        k[1] = 0;
        w    = winkel(k, v1);
        if(v1[1] < 0)w = 2*Math.PI-w;
        return w;
    }

    static double winkel(double[] v1, double[] v2){
        //Berechnet den Winkel zwischen zwei Vektoren in winkelRad aus [0,180]

        double betrag_v1   = Math.sqrt(Math.pow(v1[0], 2)+Math.pow(v1[1], 2));
        double betrag_v2   = Math.sqrt(Math.pow(v2[0], 2)+Math.pow(v2[1], 2));
        double winkelGrad;
        double winkelRad;
        double skalPro;

        if(betrag_v1==0 || betrag_v2==0){
            winkelGrad = 0;
            winkelRad  = 0;

        }
        else{
            skalPro    = (v1[0]*v2[0])+(v1[1]*v2[1]);
            winkelRad  = skalPro/(betrag_v1*betrag_v2);
            if(winkelRad> 1)winkelRad= 1;
            if(winkelRad<-1)winkelRad=-1;
            winkelRad  = Math.acos(winkelRad);
            winkelGrad = winkelRad*180/Math.PI;
        }


        return winkelRad;
    }


    double[] folgen(ArrayList<Vehicle> all){
        double[] pos_dest   = new double[2];
        double[] vel_dest   = new double[2];
        double[] acc_dest   = new double[2];
        acc_dest[0]         = 0;
        acc_dest[1]         = 0;
        Vehicle v = null;

        if(type == 0){
            for(int i=0;i<all.size();i++){
                v = all.get(i);
                if(v.type == 1)break;
            }
            double dist = Math.sqrt(Math.pow(v.pos[0]-this.pos[0],2) + Math.pow(v.pos[1]-this.pos[1],2));

            if(dist < rad_fol && inFront(v)){
                double[] pkt  = new double[2];
                double[] ort1 = new double[2];
                double[] ort2 = new double[2];
                double[] ort3 = new double[2];
                pkt[0]      = pos[0];
                pkt[1]      = pos[1];
                ort1[0]     = v.pos[0];
                ort1[1]     = v.pos[1];
                ort2[0]     = v.pos[0]+(rad_fol*v.vel[0]);
                ort2[1]     = v.pos[1]+(rad_fol*v.vel[1]);
                ort3        = punktVektorMINAbstand_punkt(pkt,  ort1, ort2);

                vel_dest[0] = pos[0]-ort3[0];//UUU
                vel_dest[1] = pos[1]-ort3[1];//III


                vel_dest    = normalize(vel_dest);
                vel_dest[0] = vel_dest[0]*max_vel;
                vel_dest[1] = vel_dest[1]*max_vel;

//				vel_dest[0] = pos[0]-v.pos[0];
//				vel_dest[1] = pos[1]-v.pos[1];

                acc_dest[0] = vel_dest[0]-vel[0];
                acc_dest[1] = vel_dest[1]-vel[1];
            }
            else if(dist < rad_fol && !inFront(v)){
                pos_dest[0] = v.pos[0]+v.vel[0];
                pos_dest[1] = v.pos[1]+v.vel[0];
                vel_dest[0] = pos_dest[0]-pos[0];
                vel_dest[1] = pos_dest[1]-pos[1];
                vel_dest    = normalize(vel_dest);
                vel_dest[0] = vel_dest[0]*max_vel;
                vel_dest[1] = vel_dest[1]*max_vel;
                acc_dest[0] = vel_dest[0]-vel[0];
                acc_dest[1] = vel_dest[1]-vel[1];
            }
            else{
                acc_dest = zusammenbleiben(all);
            }
        }

        return acc_dest;
    }


    boolean inFront(Vehicle v){
        //
        boolean erg  = false;
        double[] tmp = new double[2];
        tmp[0] = pos[0]-v.pos[0];
        tmp[1] = pos[1]-v.pos[1];

        if(winkel(tmp, v.vel)<Math.PI/6)erg = true;
        else                            erg = false;

        return erg;
    }

    boolean inFront(Vehicle v, double[] temp){
        //
        boolean erg  = false;
        double[] tmp = new double[2];
        tmp[0] = pos[0]-temp[0];
        tmp[1] = pos[1]-temp[1];

        if(winkel(tmp, v.vel)<Math.PI/6)erg = true;
        else                            erg = false;

        return erg;
    }


    double[] punktVektorMINAbstand_punkt(double[] pkt, double[] ort1, double[] ort2){
        //berechnet denjenigen Punkt abstandsPkt auf einer Geraden [ort1, ort2], mit k�rzester Entfernung zum geg. Punkt pkt
        double[] abstandsPkt = new double[2];
        abstandsPkt[0]       = 0;
        abstandsPkt[1]       = 0;

        double dist;
        double winkel1;
        double winkel2;

        double[] richtung1 = new double[2];
        double[] richtung2 = new double[2];

        richtung1[0] = ort2[0]-ort1[0];
        richtung1[1] = ort2[1]-ort1[1];
        richtung2[0] = pkt[0] -ort1[0];
        richtung2[1] = pkt[1] -ort1[1];
        winkel1      = winkel(richtung1, richtung2);
        richtung1[0] = ort1[0]-ort2[0];
        richtung1[1] = ort1[1]-ort2[1];
        richtung2[0] = pkt[0] -ort2[0];
        richtung2[1] = pkt[1] -ort2[1];
        winkel2      = winkel(richtung1, richtung2);

        if(winkel1>=Math.PI/2){
            abstandsPkt[0] = ort1[0];
            abstandsPkt[1] = ort1[1];
        }
        else if(winkel2>=Math.PI/2){
            abstandsPkt[0] = ort2[0];
            abstandsPkt[1] = ort2[1];
        }
        else{
            richtung1[0] = ort2[0]-ort1[0];
            richtung1[1] = ort2[1]-ort1[1];
            richtung2[0] = pkt[0] -ort1[0];
            richtung2[1] = pkt[1] -ort1[1];
            winkel1      = winkel(richtung1, richtung2);
            dist         = length(richtung2);
            double lot   = dist  * Math.cos(winkel1);
            double[] lotPkt = normalize(richtung1);
            abstandsPkt[0]  = ort1[0] + lot*lotPkt[0];
            abstandsPkt[1]  = ort1[1] + lot*lotPkt[1];
        }

        return abstandsPkt;
    }





}


