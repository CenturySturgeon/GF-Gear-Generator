#Author-Juan Miguel Gras Olea
#Gear Generator for basic high quality gears
import adsk.core, adsk.fusion, adsk.cam, traceback
import math as mt

defaultfc=True
handlers=[]
list=['0.4 mm','0.5 mm','0.6 mm','0.7 mm','0.8 mm','0.9 mm','1 mm','1.25 mm','1.5 mm','1.75 mm','2 mm','2.25 mm','2.5 mm','2.75 mm','3 mm','3.25 mm','3.5 mm','3.75 mm','4 mm','4.25 mm','4.5 mm','4.75 mm','5 mm','5.25 mm','5.5 mm','5.75 mm','6 mm','6.5 mm','7 mm','8 mm','9 mm','10 mm','11 mm','12 mm','13 mm','14 mm','15 mm','16 mm','18 mm','20 mm','22 mm','24 mm','27 mm','30 mm','33 mm','36 mm','39 mm','42 mm','45 mm','50 mm','60 mm','65 mm','70 mm','75 mm']
def linspace(inicio,fin,separaciones):
    lista=[]
    a=(fin-inicio)/(separaciones-1)
    for i in range(0,separaciones):
        lista.append(inicio+a*i)
    return lista

def radToDeg(x):
    grados=x*180/mt.pi
    return grados

def htl(numerop):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    tgrupos = design.timeline.timelineGroups
    if design.timeline.count >= (numerop+1):
        index = design.timeline.count - numerop
    else:
        index = 0
    tgrupos.add(index, design.timeline.count - 1)

def parameters(m,z,ap,ah,anchoeng,bool,X,fastcompute):
    dp=m*z
    rp=dp/2
    db=dp*mt.cos(ap)
    rb=db/2
    da=dp+2*m
    ra=da/2
    df=dp-2.5*m
    rf=df/2
    T=mt.pi*m/2
    vm=X*m
    dv=dp+2*X*m
    dva=dv+2*m
    dvf=dv-2.5*m
    rv=dv/2
    rva=dva/2
    rvf=dvf/2
    alpha=((mt.sqrt((dp**2)-(db**2)))/db)-ap
    beta=mt.pi/(2*z)
    def inv(x):
        iv=mt.tan(x)-x
        return iv
    def Tt(d):
        at=mt.acos(rb/(d/2))
        T1=d*((T/dp)+inv(ap)-inv(at))
        Teta=T1/(d/2)
        return Teta
    if fastcompute==True:
        aok=50
    elif m<15 and fastcompute==False:
        aok=int((89*mt.sqrt(m/.3))-3)
    elif m>=15 and fastcompute==False:
        aok=425
    angrot2=2*mt.pi-alpha-beta
    angrot=(mt.pi/(z))-alpha-beta
    u=linspace(0,mt.sqrt(((da/db)**2)-1),aok)
    v=linspace(0,mt.sqrt(((da/db)**2)-1),aok)
    x=[]
    y=[]
    x2=[]
    y2=[]
    for i in range(0,len(u)):
        x.append(rb*(mt.cos(u[i]+angrot2)+u[i]*mt.sin(u[i]+angrot2)))
        y.append(rb*(mt.sin(u[i]+angrot2)-u[i]*mt.cos(u[i]+angrot2)))
        x2.append(rb*(mt.cos(v[i]+angrot2)+v[i]*mt.sin(v[i]+angrot2)))
        y2.append(-rb*(mt.sin(v[i]+angrot2)-v[i]*mt.cos(v[i]+angrot2)))
    al=[]
    bl=[]
    cl=[]
    dl=[]
    if X>0 or X<0:
        for i in range(0,len(u)):
            al.append(rb*(mt.cos(u[i]+angrot)+u[i]*mt.sin(u[i]+angrot))+vm)
            bl.append(rb*(mt.sin(u[i]+angrot)-u[i]*mt.cos(u[i]+angrot)))
            cl.append(rb*(mt.cos(v[i]+angrot)+v[i]*mt.sin(v[i]+angrot))+vm)
            dl.append(-rb*(mt.sin(v[i]+angrot)-v[i]*mt.cos(v[i]+angrot)))
        x=al
        y=bl
        x2=cl
        y2=dl
    #creo la helice
    vool=1
    if bool==True:
        vool=-1
    try:
        ph=mt.pi*dp*mt.cos(ah)/mt.sin(ah)
        aph=10*anchoeng*2*mt.pi/ph
        t2=linspace(0,aph,aok)
        bb=ph/(2*mt.pi)
        zl=[]
        xl=[]
        yl=[]
        xl2=[]
        yl2=[]
        for i in range(0,len(t2)):
            zl.append(bb*t2[i])
            xl.append(rp*mt.cos(t2[i]+mt.pi)+dp)
            yl.append(rp*mt.sin(-vool*t2[i]))
            xl2.append(rp*mt.cos(t2[i]))
            yl2.append(rp*mt.sin(vool*t2[i]))
    except:
        ph=0
        aph=0
        t2=linspace(0,0,aok)
        zl=t2
        xl=t2
        yl=t2
        xl2=t2
        yl2=t2
    xrot=[]
    yrot=[]
    x2rot=[]
    y2rot=[]
    for i in range(0,len(u)):
        xrot.append(rb*(mt.cos(u[i]+angrot2+vool*aph)+u[i]*mt.sin(u[i]+angrot2+vool*aph))+(dp-2*(rp*mt.cos(aph))))
        yrot.append(rb*(mt.sin(u[i]+angrot2+vool*aph)-u[i]*mt.cos(u[i]+angrot2+vool*aph))+2*rp*mt.sin(-vool*aph))
        x2rot.append(rb*(mt.cos(v[i]+angrot2-vool*aph)+v[i]*mt.sin(v[i]+angrot2-vool*aph))+(dp-2*(rp*mt.cos(aph))))
        y2rot.append(-rb*(mt.sin(v[i]+angrot2-vool*aph)-v[i]*mt.cos(v[i]+angrot2-vool*aph))+2*rp*mt.sin(-vool*aph))
    try:
        lizq=mt.sqrt(2*(dp**2)*(1-mt.cos(-vool*aph)))
        uang=mt.asin(dp*mt.sin(-vool*aph)/lizq)
        xneorig=lizq*mt.cos(uang)
        yneorig=dp*mt.sin(-vool*aph)
    except:
        xneorig=0
        yneorig=0
    xrot2=[]
    yrot2=[]
    y3rot=[]
    x3rot=[]
    for i in range(0,len(u)):
        xrot2.append(rb*(mt.cos(u[i]+angrot2+vool*aph)+u[i]*mt.sin(u[i]+angrot2+vool*aph)))
        yrot2.append(rb*(mt.sin(u[i]+angrot2+vool*aph)-u[i]*mt.cos(u[i]+angrot2+vool*aph)))
        x3rot.append(rb*(mt.cos(v[i]+angrot2-vool*aph)+v[i]*mt.sin(v[i]+angrot2-vool*aph)))
        y3rot.append(-rb*(mt.sin(v[i]+angrot2-vool*aph)-v[i]*mt.cos(v[i]+angrot2-vool*aph)))
    return rf,x,y,x2,y2,aok,Tt(da),ra,angrot2,alpha,beta,xneorig,yneorig,zl,xl,yl,xrot,yrot,x2rot,y2rot,xl2,yl2,xrot2,yrot2,x3rot,y3rot,aph,rb,rva,rvf
#parameters obtiene y arroja los parámetros indispensables para generar el perfil del diente, la hélice etc.

def skeng1(m,ap,rf,ra,x,y,x2,y2,aok,Ttda,escorona,esStdr,espesorc,newComp):
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        rootComp = design.rootComponent
        rootComp = newComp
        rcor=ra+espesorc
        sketch=rootComp.sketches.add(rootComp.xYConstructionPlane)
        orig=adsk.core.Point3D.create(0,0,0)
        orig2=adsk.core.Point3D.create((rf+ra)/10,0,0)
        extrudes=rootComp.features.extrudeFeatures
        sketch2=rootComp.sketches.add(rootComp.xYConstructionPlane)

        circles=sketch2.sketchCurves.sketchCircles
        circles1=sketch.sketchCurves.sketchCircles
        dp=2*ra-2*m
        db=dp*mt.cos(ap)
        v=0
        if db<=2*rf:
            v=2*rf-db
        if escorona==True and esStdr==True:
            circrf=circles.addByCenterRadius(orig,rcor/10)
            circra=circles.addByCenterRadius(orig,rf/10)
        if escorona==True and esStdr==False:
            circra=circles.addByCenterRadius(orig2,(ra+v)/10)
            circrcor=circles.addByCenterRadius(orig2,rcor/10)
        else:
            circrf=circles.addByCenterRadius(orig,rf/10)
        if esStdr==True:
            prof2=sketch2.profiles.item(0)
        elif esStdr==False:
            prof2=sketch2.profiles.item(1)
        # hacer grupos de puntos
        points=adsk.core.ObjectCollection.create()
        points2=adsk.core.ObjectCollection.create()
        puntos=adsk.core.ObjectCollection.create()
        puntos2=adsk.core.ObjectCollection.create()
        puntos.add(adsk.core.Point3D.create(x[0]/10,y[0]/10,0))
        puntos.add(adsk.core.Point3D.create(x[1]/10,y[1]/10,0))
        puntos2.add(adsk.core.Point3D.create(x2[0]/10,y2[0]/10,0))
        puntos2.add(adsk.core.Point3D.create(x2[1]/10,y2[1]/10,0))
        # spa y spb crean splines a través de los puntos de involuta
        # spc y spd son splines del primer punto de la involuta al segundo, ya que se genera un error de hacerla corrida
        for i in range(1, aok):
            points.add(adsk.core.Point3D.create(x[i]/10,y[i]/10,0))
            points2.add(adsk.core.Point3D.create(x2[i]/10,y2[i]/10,0))
        spa=sketch.sketchCurves.sketchFittedSplines.add(points)
        spb=sketch.sketchCurves.sketchFittedSplines.add(points2)
        spc=sketch.sketchCurves.sketchFittedSplines.add(puntos)
        spd=sketch.sketchCurves.sketchFittedSplines.add(puntos2)
        # lineas para el perfil del diente
        lines=sketch.sketchCurves.sketchLines
        line1=lines.addByTwoPoints(adsk.core.Point3D.create(0,0,0),puntos[0])
        line2=lines.addByTwoPoints(adsk.core.Point3D.create(0,0,0),puntos2[0])
        # crear punto de inicio para el arco
        pointo = adsk.core.Point3D.create(x[aok-1]/10,y[aok-1]/10,0)
        # crear arco en el da
        sketch.sketchCurves.sketchArcs.addByCenterStartSweep(orig,pointo,Ttda)
        if esStdr==False:
            # Por algun motivo, debo agregar dos circulos para que Fusion note la diferencia entre los perfiles (a pesar de que si los cuenta bien)
            circles.addByCenterRadius(orig2,(ra+v)/10)
        if esStdr==True:
            prof=sketch.profiles.item(0)
        elif esStdr==False:
            prof=sketch.profiles.item(0)
        return prof2,prof,sketch
#skeng1 realiza el sketch base para el cilindro de extrusión y el perfil del diente

def skeng2(x,y,x2,y2,rva,rvf,aok,rb,m,z,ap,X, newComp):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        rootComp = newComp
        sketch=rootComp.sketches.add(rootComp.xYConstructionPlane)

        orig=adsk.core.Point3D.create(0,0,0)
        planes=rootComp.constructionPlanes
        extrudes=rootComp.features.extrudeFeatures
        sketch2=rootComp.sketches.add(rootComp.xYConstructionPlane)

        sweeps = rootComp.features.sweepFeatures
        circles=sketch2.sketchCurves.sketchCircles
        circles1=sketch.sketchCurves.sketchCircles


        dp = m * z
        dvp=dp+2*X*m
        rvf=(dvp-2.5*m)/2
        db = dp * mt.cos(ap)
        alpa=mt.sqrt((dp*dp)-(db*db))/db-ap
        alpa2=mt.pi/z-(alpa+mt.pi/(2*z))
        df=dp-2.5*m
        rf=df/2
        adesc=mt.atan((rb*mt.sin(alpa2))/(rb*mt.cos(alpa2)+X*m))

        coordenx=rvf*mt.cos(adesc)
        coordeny=rvf*mt.sin(adesc)
        lux = adsk.core.Point3D.create(coordenx/10, coordeny/ 10, 0)
        rascahuele=adsk.core.Point3D.create(coordenx / 10, -coordeny / 10, 0)
        sketch.sketchCurves.sketchArcs.addByCenterStartSweep(orig,rascahuele,2*adesc)


        # hacer grupos de puntos
        points = adsk.core.ObjectCollection.create()
        points2 = adsk.core.ObjectCollection.create()
        puntos = adsk.core.ObjectCollection.create()
        puntos2 = adsk.core.ObjectCollection.create()
        puntos.add(adsk.core.Point3D.create(x[0]/10,y[0]/10,0))
        puntos.add(adsk.core.Point3D.create(x[1]/10,y[1]/10,0))
        puntos2.add(adsk.core.Point3D.create(x2[0]/10,y2[0]/10,0))
        puntos2.add(adsk.core.Point3D.create(x2[1]/10,y2[1]/10,0))
        # spa y spb crean splines a través de los puntos de involuta
        # spc y spd son splines del primer punto de la involuta al segundo, ya que se genera un error de hacerla corrida
        for i in range(1, aok):
            points.add(adsk.core.Point3D.create(x[i]/10,y[i]/10,0))
            points2.add(adsk.core.Point3D.create(x2[i]/10,y2[i]/10,0))
        spa = sketch.sketchCurves.sketchFittedSplines.add(points)
        spb = sketch.sketchCurves.sketchFittedSplines.add(points2)
        spc = sketch.sketchCurves.sketchFittedSplines.add(puntos)
        spd = sketch.sketchCurves.sketchFittedSplines.add(puntos2)
        # lineas para el perfil del diente
        lines = sketch.sketchCurves.sketchLines
        #lines2=sketch2.sketchCurves.sketchLines
        line1 = lines.addByTwoPoints(lux, puntos[0])
        line2 = lines.addByTwoPoints(rascahuele, puntos2[0])
        # crear punto de inicio para el arco
        pointo = adsk.core.Point3D.create(x[aok-1]/10,y[aok-1]/10, 0)
        pointo2 = adsk.core.Point3D.create(x2[aok-1]/10,y2[aok-1]/10,0)
        puntoarc=adsk.core.Point3D.create((rva+1)/10,0,0)
        # crear arco en el da
        sketch.sketchCurves.sketchArcs.addByThreePoints(pointo2,puntoarc,pointo)
        sketcht=rootComp.sketches.item(0)
        vec=adsk.core.Vector3D.create(0,1,0)
        vec.add(adsk.core.Vector3D.create(0,10,0))
        objc=adsk.core.ObjectCollection.create()
        #circles1.addByCenterRadius(orig,rvf/10)
        #c=2
        #if rb<=rvf:
            #c=1
        prof=sketch.profiles.item(0)
        circles.addByCenterRadius(orig,rva/10)
        prof2=sketch2.profiles.item(0)

        return prof2,prof,sketch
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
def extruir(perfil,anchoeng,newComp,u='Escribe: NewBody,Join o Cut'):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        rootComp = newComp
        if u=='NewBody':
            operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        elif u=='Join':
            operation=adsk.fusion.FeatureOperations.JoinFeatureOperation
        elif u=='Cut':
            operation=adsk.fusion.FeatureOperations.CutFeatureOperation
        extrudes=rootComp.features.extrudeFeatures
        extInput=extrudes.createInput(perfil,operation)
        extInput.setDistanceExtent(False,adsk.core.ValueInput.createByReal(anchoeng))
        extrusion=extrudes.add(extInput)
        try:
            cara=extrusion.endFaces.item(0)

            return extrusion,cara
        except:
            return extrusion
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#extruir extruye un  perfil determinado y se puede seleccionar si esta extrusión sera de corte, combinada o un nuevo cuerpo

def cpattern(linecenter,esconico,ra,rf,z,diente,esStdr,anchoeng,newComp,u='Escribe si la operación fue Cut o Join para el perfil del diente'):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = newComp
    if esStdr==False and esconico==False:
        sketch=rootComp.sketches.add(rootComp.xYConstructionPlane)
        lines=sketch.sketchCurves.sketchLines
        zline=lines.addByTwoPoints(adsk.core.Point3D.create((ra+rf)/10,0,0),adsk.core.Point3D.create((ra+rf)/10,0,anchoeng))
        eje=zline
        sketch.isVisible=False
    elif esconico==True and esStdr==True:
        eje=linecenter
    elif esconico==False and esStdr==True:
        zAxis=rootComp.zConstructionAxis
        eje=zAxis
    inputEntites=adsk.core.ObjectCollection.create()
    inputEntites.add(diente)
    circularFeats=rootComp.features.circularPatternFeatures
    circularFeatInput=circularFeats.createInput(inputEntites,eje)
    circularFeatInput.quantity=adsk.core.ValueInput.createByReal(z)
    circularFeatInput.totalAngle=adsk.core.ValueInput.createByString('360 deg')
    circularFeatInput.isSymmetric=False
    a=0
    if u=='Cut':
        a=1
    circularFeatInput.patternComputeOption=a
    circularFeat=circularFeats.add(circularFeatInput)


#cpattern crea un patrón circular de cuerpos o features.'esStdr' es para especificar si es una corona distanciada del origen y 'u' es porque las operaciones de corte
#tienen generan problemas para una compute option de identical(a=1) y por lo tanto debe usarse una de adjust (a=2) aunque sea mas lenta

def helixext(esPS,ra,rf,escorona,aph,sketch,aok,xl,yl,zl,anchoeng,cw,newComp,u='Escribe: NewBody,Join o Cut'):
    try:
        app=adsk.core.Application.get()
        ui=app.userInterface
        design=app.activeProduct
        rootComp=newComp
        sweeps=rootComp.features.sweepFeatures
        lines=sketch.sketchCurves.sketchLines
        pvec=adsk.core.Point3D.create(0,0,anchoeng/1.25)
        orig = adsk.core.Point3D.create(0,0,0)

        m=(ra-rf)/2.25
        rp=ra-m
        pah=mt.pi*(2*rp)*(mt.cos(aph)/mt.sin(aph))
        regla3=(10*(anchoeng/1.25)*2*mt.pi)/pah
        #ui.messageBox(str(regla3))
        if cw==True:
            regla3=-regla3
        c=0
        if escorona==False:
            prof=sketch.profiles.item(c)
        else:
            prof=sketch.profiles.item(c)
            orig=adsk.core.Point3D.create((ra+rf)/10,0,0)
            pvec=adsk.core.Point3D.create((ra+rf)/10,0,anchoeng/1.25)
        if u=='NewBody':
            operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        elif u=='Join':
            operation=adsk.fusion.FeatureOperations.JoinFeatureOperation
        elif u=='Cut':
            operation=adsk.fusion.FeatureOperations.CutFeatureOperation
        #for i in range(0,aok):
            #helice.add(adsk.core.Point3D.create(xl[i]/10,yl[i]/10,zl[i]/10))
        #helicl=sketch.sketchCurves.sketchFittedSplines.add(helice)
        #path=adsk.fusion.Path.create(helicl,0)
        lpath=lines.addByTwoPoints(orig, pvec)
        #path2=adsk.fusion.Path.create(lpath, 0)
        path2=newComp.features.createPath(lpath, False)
        esq= rootComp.sketches.item(rootComp.sketches.count-2)
        prof=esq.profiles.item(0)
        sweepInput=sweeps.createInput(prof,path2,operation)
        #sweepInput.path=path2
        sweepInput.twistAngle=adsk.core.ValueInput.createByReal(regla3)
        #sweepInput.guideRail=path
        sweep=sweeps.add(sweepInput)
        return sweep,u
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#helixt extruye un perfil (sketch) sobre la helice calculada en parameters() y se debe especificar el tipo de operacion del barrido

def excesscut(ra,rf,espesorc,anchoeng,newComp):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        rootComp = newComp
        rcor = ra + espesorc
        sketch = rootComp.sketches.add(rootComp.xYConstructionPlane)
        orig = adsk.core.Point3D.create(0, 0, 0)
        orig2 = adsk.core.Point3D.create((rf + ra) / 10, 0, 0)
        extrudes = rootComp.features.extrudeFeatures
        circles1 = sketch.sketchCurves.sketchCircles
        circles1.addByCenterRadius(orig2,(ra+rf+rcor)/10)
        circles1.addByCenterRadius(orig2,(rcor)/10)
        prof=sketch.profiles.item(0)
        extInput=extrudes.createInput(prof,adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(2*anchoeng+1))
        extrusion = extrudes.add(extInput)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#Corta el exceso de los perfiles de diente en coronas no estándar

def mirror(sketch,anchoeng,cara,newComp):
    try:
        app=adsk.core.Application.get()
        ui=app.userInterface
        design=app.activeProduct
        rootComp=newComp
        sweeps=rootComp.features.sweepFeatures
        lines=sketch.sketchCurves.sketchLines
        pvec=adsk.core.Point3D.create(0,0,anchoeng)
        orig=adsk.core.Point3D.create(0,0,0)
        orig3=adsk.core.Point3D.create(0,0,anchoeng)
        sketchLineOne=lines.addByTwoPoints(orig,orig3)
        distance=adsk.core.ValueInput.createByReal(anchoeng)
        planes=rootComp.constructionPlanes
        planeInput=planes.createInput()
        planeInput.setByOffset(rootComp.xYConstructionPlane,distance)
        if cara==False:
            uj=planes.add(planeInput)
        else:
            uj=cara

        conta=rootComp.bRepBodies.count
        TargetBody=rootComp.bRepBodies.item(conta-1)
        bodyco=adsk.core.ObjectCollection.create()
        bodyco.add(TargetBody)
        mirrorFeatures=rootComp.features.mirrorFeatures
        mirrorInput=mirrorFeatures.createInput(bodyco,uj)
        mirrorInput.patternComputeOption=0
        mirrorFeatures.add(mirrorInput)
        Toolbodies=adsk.core.ObjectCollection.create()
        conta2=rootComp.bRepBodies.count
        Toolbodies.add(rootComp.bRepBodies.item(conta2-1))
        CombineInput=rootComp.features.combineFeatures.createInput(TargetBody,Toolbodies)
        CombineInput.operation=0
        CombineInput.isKeepToolBodies=False
        CombineInput.isNewComponent=False
        rootComp.features.combineFeatures.add(CombineInput)
        sketch.isVisible=False
        #uj.isVisible=False
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#mirror crea un plano y realiza una simetría de cuerpo respecto al mismo

def planecut(espesorc,ra,rf,anchoeng,esStdr):
    try:
        app=adsk.core.Application.get()
        ui=app.userInterface
        design=app.activeProduct
        rootComp=design.rootComponent
        planes=rootComp.constructionPlanes
        extrudes=rootComp.features.extrudeFeatures
        planeInput=planes.createInput()
        orig2=adsk.core.Point3D.create((ra+rf)/10,0,0)
        orig=adsk.core.Point3D.create(0,0,0)
        origen=orig
        if esStdr==True:
            origen=orig2
        distance=adsk.core.ValueInput.createByReal(1.25*anchoeng+1)
        planeInput.setByOffset(rootComp.xYConstructionPlane,distance)
        uj=planes.add(planeInput)
        sketch=rootComp.sketches.add(uj)
        circles=sketch.sketchCurves.sketchCircles
        circles.addByCenterRadius(origen,(ra+espesorc)/10)
        extInput=extrudes.createInput(sketch.profiles.item(0),adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-.25*anchoeng-1))
        extrudes.add(extInput)
        conta=rootComp.bRepBodies.count
        diente=rootComp.bRepBodies.item(conta-1)
        uj.isVisible=False
        return diente
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#planecut crea un plano y un sketch con un circulo que después corta el sobrante del perfil del diente (por eso se extruye 1.25*anchoeng)
#Es necesario que el plano quede un poco arriba del diente para que al cortarlo no genere problemas

def combine(z,newComp):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        rootComp = newComp
        conta=(rootComp.bRepBodies.count)-1
        conta2=rootComp.bRepBodies.count
        Targetbody=rootComp.bRepBodies.item(conta-z)
        Toolbodies=adsk.core.ObjectCollection.create()
        for i in range (conta2-z,conta2):
            Toolbodies.add(rootComp.bRepBodies.item(i))
        CombineInput=rootComp.features.combineFeatures.createInput(Targetbody,Toolbodies)
        CombineInput.operation=0
        CombineInput.isKeepToolBodies=False
        CombineInput.isNewComponent=False
        rootComp.features.combineFeatures.add(CombineInput)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#combine combina todos los perfiles de los dientes helicoidales con el cilindro notese que los 'conta' son utilizaos para poder crear engranes cuando ya hay otras piezas

def crwoncut(anchoeng,rb,rf,ra,espesorc, newComp):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        rootComp = newComp
        extrudes=rootComp.features.extrudeFeatures
        orig=adsk.core.Point3D.create(0,0,0)
        orig2=adsk.core.Point3D.create((ra+rf)/10,0,0)
        skethc=rootComp.sketches.add(rootComp.xYConstructionPlane)
        circles=skethc.sketchCurves.sketchCircles
        rcor=(ra+espesorc)/10
        circles.addByCenterRadius(orig2,(rcor)/1)
        circles.addByCenterRadius(orig2,(rcor+(2*rf-2*rb)/10)/1)
        prof=skethc.profiles.item(1)
        extInput=extrudes.createInput(prof,adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setDistanceExtent(False,adsk.core.ValueInput.createByReal(2*anchoeng))
        extrudes.add(extInput)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#Fusion tiene problemas para seleccionar el perfil correcto del diente cuando el circulo del sketch que uso para que no seleccione todo el perfil (en engranes no stdr)
#interrseciona a la involuta por eso en skeng hay una condición basada en el db y df por lo que si el espesor radial de la corona no stdr es chico tiene que usarse
#un corte para eliminar el sobrante

def rev(prof, eje, newComp, u='Escribe: NewBody,Join o Cut'):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        rootComp = newComp
        revolvefeats = rootComp.features.revolveFeatures
        if u=='NewBody':
            operation = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        elif u=='Join':
            operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        elif u=='Cut':
            operation = adsk.fusion.FeatureOperations.CutFeatureOperation

        revin4=revolvefeats.createInput(prof,eje,operation)
        angle4=adsk.core.ValueInput.createByReal(2*mt.pi)
        revin4.setAngleExtent(False,angle4)
        revu=revolvefeats.add(revin4)
        cuenta=rootComp.bRepBodies.count
        diente=rootComp.bRepBodies.item(cuenta-cuenta)
        return diente
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#rev crea una revolución de un perfil ya hecho sobre un eje dado

def sketchcon(x,y,x2,y2,z,z2,rp,rp2,rf,ra,Ttda,m,aok, newComp):
    try:
        aconico=mt.atan(z/z2)
        h=2.25*m
        altura=5*m
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        rootComp = newComp
        sketch = rootComp.sketches.add(rootComp.xYConstructionPlane)
        orig = adsk.core.Point3D.create(0,0,0)
        planes = rootComp.constructionPlanes
        orig2 = adsk.core.Point3D.create((rf+ra)/10,0,0)
        extrudes = rootComp.features.extrudeFeatures
        # sketch2=rootComp.sketches.add(rootComp.xYConstructionPlane)
        sweeps = rootComp.features.sweepFeatures
        # circles=sketch2.sketchCurves.sketchCircles
        circles1 = sketch.sketchCurves.sketchCircles
        # circrf=circles1.addByCenterRadius(orig,rf/10)
        # hacer grupos de puntos
        points = adsk.core.ObjectCollection.create()
        points2 = adsk.core.ObjectCollection.create()
        puntos = adsk.core.ObjectCollection.create()
        puntos2 = adsk.core.ObjectCollection.create()
        puntos.add(adsk.core.Point3D.create(x[0]/10, y[0]/10,0))
        puntos.add(adsk.core.Point3D.create(x[1]/10, y[1]/10,0))
        puntos2.add(adsk.core.Point3D.create(x2[0]/10,y2[0]/10,0))
        puntos2.add(adsk.core.Point3D.create(x2[1]/10,y2[1]/10,0))
        # spa y spb crean splines a través de los puntos de involuta
        # spc y spd son splines del primer punto de la involuta al segundo, ya que se genera un error de hacerla corrida
        for i in range(1, aok):
            points.add(adsk.core.Point3D.create(x[i]/10,y[i]/10,0))
            points2.add(adsk.core.Point3D.create(x2[i]/10,y2[i]/10,0))
        spa = sketch.sketchCurves.sketchFittedSplines.add(points)
        spb = sketch.sketchCurves.sketchFittedSplines.add(points2)
        spc = sketch.sketchCurves.sketchFittedSplines.add(puntos)
        spd = sketch.sketchCurves.sketchFittedSplines.add(puntos2)
        # lineas para el perfil del diente
        lines = sketch.sketchCurves.sketchLines
        # lines2=sketch2.sketchCurves.sketchLines
        line1 = lines.addByTwoPoints(adsk.core.Point3D.create(0,0,0),puntos[0])
        line2 = lines.addByTwoPoints(adsk.core.Point3D.create(0,0,0),puntos2[0])
        # crear punto de inicio para el arco
        pointo = adsk.core.Point3D.create(x[aok-1]/10,y[aok-1]/10,0)
        # crear arco en el da
        sketch.sketchCurves.sketchArcs.addByCenterStartSweep(orig,pointo,Ttda)
        sketcht = rootComp.sketches.item(0)
        vec=adsk.core.Vector3D.create(0, 1, 0)
        vec.add(adsk.core.Vector3D.create(0, 10, 0))
        objc=adsk.core.ObjectCollection.create()
        # circles1.addByCenterRadius(orig,rf/10)
        prof=sketch.profiles.item(0)

        largot=rp+1.25*m
        puntosup=adsk.core.Point3D.create((rp-rp*mt.cos(aconico))/10,0,(rp*mt.sin(aconico))/10)
        puntocon=adsk.core.Point3D.create(((rp-rp*mt.cos(aconico))+rp2*mt.cos(mt.pi/2-aconico))/10,0,(rp*mt.sin(aconico)+rp2*mt.sin(mt.pi/2-aconico))/10)
        puntoad=adsk.core.Point3D.create((ra)/10,0,0)
        puntop=adsk.core.Point3D.create((rp)/10,0,0)
        puntodd=adsk.core.Point3D.create((rf)/10,0,0)
        # lines.addByTwoPoints(puntosup,puntop)
        linead=lines.addByTwoPoints(puntocon, puntoad)
        lines.addByTwoPoints(puntocon, puntodd)
        linepp=lines.addByTwoPoints(puntocon,puntop)
        linecenter=lines.addByTwoPoints(puntosup,puntocon)

        path=newComp.features.createPath(linepp, False)
        guiderail=newComp.features.createPath(linead, False)

        Ao=mt.sqrt((rp**2)+(rp2**2))
        F=Ao/3

        sketch3 = rootComp.sketches.add(rootComp.xYConstructionPlane)
        lines3 = sketch3.sketchCurves.sketchLines
        puntodisco = adsk.core.Point3D.create((rf-(rp-1.25*m*mt.cos(aconico))*mt.cos(aconico))/10,0,((rp-1.25*m*mt.cos(aconico))*mt.sin(aconico))/10)
        lines3.addByTwoPoints(puntodd, puntodisco)
        lines3.addByTwoPoints(puntodisco, puntocon)
        angulof = mt.atan((rp2+1.25*m*mt.sin(aconico)) / (rp - 1.25 * m * mt.cos(aconico)))
        puntodisco2 = adsk.core.Point3D.create((rf-F*mt.cos(aconico + angulof))/10,0,F/10)
        lines3.addByTwoPoints(puntodisco2, puntodd)
        puntodisco3 = adsk.core.Point3D.create(((rf-F*mt.cos(aconico+angulof))-2*rf*mt.cos(aconico))/10,0,((F)+2*rf*mt.sin(aconico))/10)
        lines3.addByTwoPoints(puntodisco2, puntodisco3)
        prof3 = sketch3.profiles.item(0)

        sketch4 = rootComp.sketches.add(rootComp.xYConstructionPlane)
        lines4 = sketch4.sketchCurves.sketchLines
        puntoram = adsk.core.Point3D.create((ra+5)/10,0,0)
        lines4.addByTwoPoints(puntodd, puntoram)
        lines4.addByTwoPoints(orig, puntodisco)
        lines4.addByTwoPoints(puntodisco, puntodd)
        puntoabaj = adsk.core.Point3D.create(0, 0, -.2)
        lines4.addByTwoPoints(puntoabaj, orig)
        puntoram2 = adsk.core.Point3D.create((ra+5)/10,0,-.2)
        lines4.addByTwoPoints(puntoabaj, puntoram2)
        lines4.addByTwoPoints(puntoram2, puntoram)

        punton = adsk.core.Point3D.create((ra+5)/10,0,F/10)
        lines4.addByTwoPoints(punton, puntodisco2)
        lines4.addByTwoPoints(punton, puntocon)
        lines4.addByTwoPoints(puntodisco, puntocon)
        lines4.addByTwoPoints(puntodisco2, puntodisco3)
        prof4 = sketch4.profiles.item(1)
        profe = adsk.core.ObjectCollection.create()
        profe.add(prof4)
        profe42=sketch4.profiles.item(0)

        return prof,path,guiderail,linepp,linead,prof3,linecenter,prof4,profe42
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#crea el sketch del perfil para los engranes cónicos

def sweep(prof,path,guiderail,linepp,linead, newComp):
    try:
        app=adsk.core.Application.get()
        ui=app.userInterface
        design=app.activeProduct
        rootComp=newComp
        sweeps=rootComp.features.sweepFeatures

        path=newComp.features.createPath(linepp, False)
        guiderail=newComp.features.createPath(linead, False)
        sweepInput=sweeps.createInput(prof, path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        sweepInput.path=path
        sweepInput.guideRail=guiderail
        sweepInput.distanceOne=adsk.core.ValueInput.createByReal(1/2)
        swep=sweeps.add(sweepInput)
        return swep
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def rotcon(rf,aconico, occ):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        rootComp = design.rootComponent

        vec=adsk.core.Vector3D.create(0, 1, 0)
        # cuento=rootComp.bRepBodies.count
        # objc=adsk.core.ObjectCollection.create()
        # objc.add(rootComp.bRepBodies.item(cuento-1))
        transform = adsk.core.Matrix3D.create()
        transform.setToRotation(-aconico,vec,adsk.core.Point3D.create((rf) / 10, 0, 0))
        occ.transform2 = transform
        # movefeats = rootComp.features.moveFeatures
        # movefeatureInput = movefeats.createInput(objc,transform)
        # movefeats.add(movefeatureInput)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#Los engranes cónicos aparecen inclinados y esta funcion los inclina para que su área de mayor contacto sea horizontal

def movebody(x,y,z):
    app=adsk.core.Application.get()
    ui=app.userInterface
    design=app.activeProduct
    rootComp=design.rootComponent

    cuento=rootComp.bRepBodies.count
    objc=adsk.core.ObjectCollection.create()
    objc.add(rootComp.bRepBodies.item(cuento-1))
    transform=adsk.core.Matrix3D.create()
    transform.translation=adsk.core.Vector3D.create(x,y,z)
    movefeats=rootComp.features.moveFeatures
    movefeatureInput=movefeats.createInput(objc,transform)
    movefeats.add(movefeatureInput)

    movefeats=rootComp.features.moveFeatures
#mueve el último cuerpo creado

def moveocc(x,y,z,occ):
    app=adsk.core.Application.get()
    ui=app.userInterface
    design=app.activeProduct
    rootComp=design.rootComponent

    transform=adsk.core.Matrix3D.create()
    transform.translation=adsk.core.Vector3D.create(x,y,z)
    occ.transform2 = transform
#Mueve el componente (por medio de la ocurrencia, el componente como tal no se puede mover)

def moveLastComponentBody(x, y, z, component):
    app=adsk.core.Application.get()
    ui=app.userInterface
    design=app.activeProduct
    rootComp=component

    cuento=rootComp.bRepBodies.count
    objc=adsk.core.ObjectCollection.create()
    objc.add(rootComp.bRepBodies.item(cuento-1))
    
    transform=adsk.core.Matrix3D.create()
    transform.translation=adsk.core.Vector3D.create(x,y,z)
    
    movefeats=rootComp.features.moveFeatures
    movefeatureInput=movefeats.createInput(objc,transform)
    movefeats.add(movefeatureInput)


def wormhelix(m,z,ap,radio,fastcompute,largotornillo,cw, newComp):
    try:
        cuerdas=round(largotornillo/(mt.pi*m))
        p=mt.pi*m
        radio=radio+1.25*m
        ah=mt.atan((mt.pi*m)/(mt.pi*2*radio))
        #if fastcompute==True:
        aok=50
        #else:
        #    aok=500
        tt=linspace(0,2*mt.pi,aok)
        bv=p/(2*mt.pi)
        coef=0
        if cw==True:
            coef=1
        zz=[]
        xx=[]
        yy=[]
        for i in range(0,len(tt)):
            zz.append(bv*tt[i])
            xx.append(radio*mt.cos(tt[i]+coef*mt.pi/2))
            yy.append(radio*mt.sin(tt[i]+coef*mt.pi/2))

        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        rootComp = newComp
        sketch5=rootComp.sketches.add(rootComp.xYConstructionPlane)
        sketch6=rootComp.sketches.add(rootComp.xYConstructionPlane)
        sketch3=rootComp.sketches.add(rootComp.xYConstructionPlane)
        orig=adsk.core.Point3D.create(0,0,0)
        extrudes=rootComp.features.extrudeFeatures



        #helix = adsk.core.ObjectCollection.create()
        #if cw==True:
        #    for i in range (0,len(tt)):
        #        helix.add(adsk.core.Point3D.create(zz[i]/10,yy[i]/10,xx[i]/10))
        #elif cw==False:
        #    for i in range (0,len(tt)):
        #        helix.add(adsk.core.Point3D.create(zz[i]/10,xx[i]/10,yy[i]/10))
        #spline=sketch3.sketchCurves.sketchFittedSplines.add(helix)

        puntoai=adsk.core.Point3D.create(-((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap)))/10,(radio+m)/10,0)
        puntoad=adsk.core.Point3D.create(((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap)))/10,(radio+m)/10,0)
        puntoabi=adsk.core.Point3D.create(-((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap)))/10,(radio-1.30*m)/10,0)
        puntoabd=adsk.core.Point3D.create(((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap)))/10,(radio-1.30*m)/10,0)

        puntoai2=adsk.core.Point3D.create(-((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap)))/10+(1*mt.pi*m/2)/10,-(radio+m)/10,0)
        puntoad2=adsk.core.Point3D.create(((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap))+(1*mt.pi*m/2))/10,-(radio+m)/10,0)
        puntoabi2=adsk.core.Point3D.create(-((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap)))/10+(1*mt.pi*m/2)/10,-(radio-1.30*m)/10,0)
        puntoabd2=adsk.core.Point3D.create(((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap))+(1*mt.pi*m/2))/10,-(radio-1.30*m)/10,0)

        puntoai3=adsk.core.Point3D.create(-((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap)))/10+(2*mt.pi*m/2)/10,(radio+m)/10,0)
        puntoad3=adsk.core.Point3D.create(((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap))+(2*mt.pi*m/2))/10,(radio+m)/10,0)
        puntoabi3=adsk.core.Point3D.create(-((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap)))/10+(2*mt.pi*m/2)/10,(radio-1.30*m)/10,0)
        puntoabd3=adsk.core.Point3D.create(((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap))+(2*mt.pi*m/2))/10,(radio-1.30*m)/10,0)

        sketch5.sketchCurves.sketchLines.addByTwoPoints(puntoai,puntoad)
        sketch5.sketchCurves.sketchLines.addByTwoPoints(puntoai,puntoabi)
        sketch5.sketchCurves.sketchLines.addByTwoPoints(puntoabi,puntoabd)
        sketch5.sketchCurves.sketchLines.addByTwoPoints(puntoabd,puntoad)
        prof1=sketch5.profiles.item(0)

        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoai2, puntoad2)
        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoai2, puntoabi2)
        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoabi2, puntoabd2)
        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoabd2, puntoad2)

        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoai3, puntoad3)
        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoai3, puntoabi3)
        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoabi3, puntoabd3)
        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoabd3, puntoad3)
        prof2=sketch6.profiles.item(0)
        prof3=sketch6.profiles.item(1)
        sketch6.isVisible=False

        sketch6=rootComp.sketches.add(rootComp.yZConstructionPlane)
        cirqulo=sketch6.sketchCurves.sketchCircles.addByCenterRadius(orig,(radio-1.25*m)/10)
        prof6=sketch6.profiles.item(0)
        extinput6=extrudes.createInput(prof6,adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extinput6.setDistanceExtent(False, adsk.core.ValueInput.createByReal((largotornillo-((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap))))/10-.001))
        cilindro=extrudes.add(extinput6)
        cara=cilindro.endFaces.item(0)
        sketch55=rootComp.sketches.add(cara)

        sketch55.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0,0),(radio+2.25*m)/10)

        extinput62=extrudes.createInput(prof6,adsk.fusion.FeatureOperations.JoinFeatureOperation)
        extinput62.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap)))/10))
        extrudes.add(extinput62)

        path=newComp.features.createPath(cirqulo,False)
        loft=rootComp.features.loftFeatures
        input=loft.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftobj = input.loftSections


        input2 = loft.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftobj2 = input2.loftSections
        if cw==False:
            loftobj.add(prof1)
            loftobj.add(prof2)
            loftobj2.add(prof2)
            loftobj2.add(prof3)
        elif cw==True:
            loftobj.add(prof2)
            loftobj.add(prof1)
            loftobj2.add(prof3)
            loftobj2.add(prof2)


        input.centerLineOrRails.addCenterLine(path)
        worm1=loft.add(input)
        input2.centerLineOrRails.addCenterLine(path)
        worm=loft.add(input2)
        length=(cuerdas*p)+((mt.pi*m/4)+(1.25*m)/(mt.tan(mt.pi/2-ap)))+-1*((mt.pi*m/4)+(1.25*m)/(mt.tan(mt.pi/2-ap)))
        sketch3.isVisible=False
        sketch5.isVisible = False
        sketch6.isVisible = False

        perfil=sketch55.profiles.item(0)
        perfil2=sketch55.profiles.item(1)



        return p, worm, cilindro,ah,length,worm1,cuerdas,perfil,perfil2
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def linearpattern(body,p,cuerdas,espacioentre, newComp):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = newComp

    inputEntites=adsk.core.ObjectCollection.create()
    inputEntites.add(body)
    quantityOne = adsk.core.ValueInput.createByString(str(cuerdas))
    xAxis=rootComp.xConstructionAxis
    distanceOne = adsk.core.ValueInput.createByReal(p/10)
    rectangularPatterns=rootComp.features.rectangularPatternFeatures
    rectangularPatternInput = rectangularPatterns.createInput(inputEntites, xAxis, quantityOne, distanceOne,adsk.fusion.PatternDistanceType.SpacingPatternDistanceType)
    rectangularFeature = rectangularPatterns.add(rectangularPatternInput)
#Crea un patrón de features sobre una línea dada

def wormcut(m,z,radio):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = design.rootComponent

    distancia=(m*z/2)+1.25*m+10*radio
    sketchz=rootComp.sketches.add(rootComp.xZConstructionPlane)
    circles=sketchz.sketchCurves.sketchCircles
    circulo=circles.addByCenterRadius(adsk.core.Point3D.create((distancia)/10,0,0),radio)

def copypaste():
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = design.rootComponent

    cuento = rootComp.bRepBodies.count
    lastbody=rootComp.bRepBodies.item(cuento-1)
    copypaste=rootComp.features.copyPasteBodies.add(lastbody)

def rowbodymove(numerodecomponentes,radioext):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = design.rootComponent

    cuento = rootComp.bRepBodies.count
    transform = adsk.core.Matrix3D.create()

    for i in range(cuento-numerodecomponentes,cuento):
        objc = adsk.core.ObjectCollection.create()
        objc.add(rootComp.bRepBodies.item(i))
        transform.translation = adsk.core.Vector3D.create(0,radioext,0)
        movefeats = rootComp.features.moveFeatures
        movefeatureInput = movefeats.createInput(objc, transform)
        movefeats.add(movefeatureInput)
#Mueve la cantidad de cuerpos (los últimos creados) especificados como conjunto en la dirección especificada

def planetgearsdr(m1,zp1,ap1,aok,anchoeng,newComp):
    list3=parameters(m1,zp1,ap1,0,anchoeng,False,0,aok)
    rf=list3[0]
    x=list3[1]
    y=list3[2]
    x2=list3[3]
    y2=list3[4]
    aok=list3[5]
    Ttda=list3[6]
    ra=list3[7]
    prof=skeng1(m1,ap1,rf,ra,x,y,x2,y2,aok,Ttda,False,True,0,newComp)
    extruir(prof[0],anchoeng,newComp,'NewBody')
    diente=extruir(prof[1],anchoeng,newComp,'NewBody')
    cpattern(1,False,ra,rf,zp1,diente[0],True,anchoeng,newComp, 'Join')
    combine(zp1,newComp)

def coronastd(aaok,z,m,ap,anchoeng,espesorc,newComp):
    list3 = parameters(m, z, ap, 0, anchoeng, False, 0, aaok)
    rf = list3[0]
    x = list3[1]
    y = list3[2]
    x2 = list3[3]
    y2 = list3[4]
    aok = list3[5]
    Ttda = list3[6]
    ra = list3[7]
    perfil = skeng1(m, ap, rf, ra, x, y, x2, y2, aok, Ttda, True, True, espesorc,newComp)
    extruir(perfil[0], anchoeng, newComp, 'NewBody')
    diente = extruir(perfil[1], anchoeng, newComp, 'Cut')
    cpattern(1, False, ra, rf, z, diente, True, anchoeng, newComp, 'Join')



def coronasnostd(aaok,z,m,anchoeng,ap,espesorc,newComp,occ):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = newComp
    list3 = parameters(m, z, ap, 0, anchoeng, False, 0, aaok)
    rf = list3[0]
    x = list3[1]
    y = list3[2]
    x2 = list3[3]
    y2 = list3[4]
    aok = list3[5]
    Ttda = list3[6]
    ra = list3[7]
    rb = list3[27]
    perfil = skeng1(m, ap, rf, ra, x, y, x2, y2, aok, Ttda, True, False, espesorc,newComp)
    esque= rootComp.sketches.item(rootComp.sketches.count-1)
    profe= esque.profiles.item(0)
    extruir(profe, anchoeng, newComp, 'NewBody')
    dienton = extruir(perfil[1], anchoeng, newComp, 'NewBody')
    diente=dienton[0]
    cpattern(1, False, ra, rf, z, diente, False, anchoeng, newComp, 'Join')
    combine(z, newComp)
    if rf >= rb and espesorc <= rf - rb:
        crwoncut(anchoeng, rb, rf, ra, espesorc, newComp)
    if (ra + espesorc) < (rf + ra):
        excesscut(ra, rf, espesorc, anchoeng, newComp)
    moveocc((-m*z)/10,0,0,occ)
    #movebody((-m*z)/10,0,0)

def helicalgs(aaok,cw,dh,z,anchoeng,m,ap,ah,newComp):
    app = adsk.core.Application.get()
    ui  = app.userInterface
    mult1 = 1
    if dh == True:
        mult1 = 2
    list3 = parameters(m, z, ap, ah, 1.25 * anchoeng, cw, 0, aaok)
    rf = list3[0]
    x = list3[1]
    y = list3[2]
    x2 = list3[3]
    y2 = list3[4]
    aok = list3[5]
    Ttda = list3[6]
    ra = list3[7]
    zl = list3[13]
    xl = list3[20]
    yl = list3[21]
    aph = list3[26]
    perfil = skeng1(m, ap, rf, ra, x, y, x2, y2, aok, Ttda, False, True, 0,newComp)
    car=extruir(perfil[0], anchoeng,newComp, 'NewBody')
    cara=car[1]
    try:
        dienton=helixext(perfil[1], ra, rf, False, ah, perfil[2], aok, xl, yl, zl, 1.25 * anchoeng,cw,newComp,'NewBody')
        diente = dienton[0]
        cpattern(1, False, ra, rf, z, diente, True, anchoeng, newComp,'Join')
        combine(z,newComp)
        if dh == True:
           mirror(perfil[2], anchoeng,False,newComp)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def coronashelstdr(aaok, cw, dh, z, anchoeng, m, ap, espesorc, ah, newComp):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = newComp
    mult1 = 1
    if dh == True:
        mult1 = 2
    list3 = parameters(m, z, ap, ah, 1.25 * anchoeng, cw, 0, aaok)
    rf = list3[0]
    x = list3[1]
    y = list3[2]
    x2 = list3[3]
    y2 = list3[4]
    aok = list3[5]
    Ttda = list3[6]
    ra = list3[7]
    zl = list3[13]
    xl = list3[20]
    yl = list3[21]
    aph = list3[26]
    perfil = skeng1(m, ap, rf, ra, x, y, x2, y2, aok, Ttda, True, True, espesorc, newComp)
    car=extruir(perfil[0], anchoeng, newComp, 'NewBody')
    cara=car[1]
    diente = helixext(False, ra, rf, False, ah, perfil[2], aok, xl, yl, zl, 1.25*anchoeng,cw, newComp, 'Cut')
    cpattern(1, False, ra, rf, z, diente[0], True, anchoeng, newComp, 'Cut')
    mainBody=rootComp.bRepBodies.item(0)

    if dh == True:
        mirror(perfil[2], anchoeng, False, newComp)
        #planos = rootComp.constructionPlanes
        #planos.item(0).isVisible = False


def coronashelnostdr(aaok,cw,dh,z,anchoeng,m,ap,espesorc,ah, newComp, occ):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = newComp
    mult1 = 1
    if dh == True:
        mult1 = 2
    list3 = parameters(m, z, ap, ah, 1.25 * anchoeng, cw, 0, aaok)
    rf = list3[0]
    x = list3[1]
    y = list3[2]
    x2 = list3[3]
    y2 = list3[4]
    aok = list3[5]
    Ttda = list3[6]
    ra = list3[7]
    zl = list3[13]
    xl = list3[14]
    yl = list3[15]
    aph = list3[26]
    rb = list3[27]
    perfil = skeng1(m, ap, rf, ra, x, y, x2, y2, aok, Ttda, True, False, espesorc, newComp)
    esque = rootComp.sketches.item(rootComp.sketches.count - 1)
    profe = esque.profiles.item(0)
    extruir(profe,anchoeng, newComp, 'NewBody')
    dienton=helixext(False, ra, rf, True, ah, perfil[2], aok, xl, yl, zl, 1.25 * anchoeng,cw, newComp, 'NewBody')
    diente= dienton[0]
    cpattern(1, False, ra, rf, z, diente, False, anchoeng, newComp, 'Join')
    combine(z, newComp)
    if dh == True:
        mirror(perfil[2], anchoeng,False, newComp)
    if rf >= rb and espesorc <= rf - rb:
        crwoncut(anchoeng, rb, rf, ra, espesorc, newComp)
    if (ra + espesorc) < (rf + ra):
        excesscut(ra, rf, espesorc, anchoeng,newComp)
    moveocc((-m*z)/10,0,0,occ)
    if rootComp.constructionPlanes.isValid==True:
        planos=rootComp.constructionPlanes
        planos.item(0).isVisible=False

def hidebodies(newComp):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = adsk.fusion.Design.cast(app.activeProduct)
    rootComp = newComp
    rootComp = design.rootComponent

    # design = adsk.fusion.Design.cast(app.activeProduct)
    # root = design.activeComponent
    # newComp = root.occurrences.count

    list=[]
    for j in range(0, rootComp.occurrences.count):
        Comp = rootComp.occurrences.item(j)
        cuento = Comp.bRepBodies.count
        for i in range(0,cuento):
            body=Comp.bRepBodies.item(i)
            list.append(body)
            body.isVisible=False
    return list
#Esconde todos los cuerpos existentes en la pestaña para evitar posibles interferencias

def showhiddenbodies(bodies,newComp):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = newComp

    for i in range(0,len(bodies)):
        bodies[i].isVisible=True
#Regresa a la vista los cuerpos ocultados por hidebodies

def Newpestana():
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = design.rootComponent
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
#Crea una nueva pestaña para alojar nuevos componentes o cuerpos

def geartext(text,ra):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = design.rootComponent
    sketch = rootComp.sketches.add(rootComp.xYConstructionPlane)
    sketchTexts = sketch.sketchTexts
    orig = adsk.core.Point3D.create(-8.5,(ra)/10,0)
    sketchTextInput = sketchTexts.createInput(text, 1.0, orig)
    sketchTextInput.textStyle = adsk.fusion.TextStyles.TextStyleBold
    sketchText = sketchTexts.add(sketchTextInput)
#crea una linea de texto centrada

def hideplanes():
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = design.rootComponent
    planos = rootComp.constructionPlanes
    cuento=planos.count
    for i in range(0,cuento):
        planos.item(i).isVisible = False

def heliwormg(m,z,ap,worm,anchoeng,vul,vul2,aaok, newComp):
    app = adsk.core.Application.get()
    ui = app.userInterface
    list3 = parameters(m, z, ap, worm[3], 1.25 * anchoeng, vul, 0, aaok)
    rf = list3[0]
    x = list3[1]
    y = list3[2]
    x2 = list3[3]
    y2 = list3[4]
    aok = list3[5]
    Ttda = list3[6]
    ra = list3[7]
    zl = list3[13]
    xl = list3[20]
    yl = list3[21]
    aph = list3[26]
    perfil = skeng1(m, ap, rf, ra, x, y, x2, y2, aok, Ttda, False, True, 0, newComp)
    extruir(perfil[0], anchoeng, newComp, 'NewBody')
    try:
        dienton= helixext(False, ra, rf, False, worm[3], perfil[2], aok, xl, yl, zl, 1.25 * anchoeng,vul, newComp, 'NewBody')
        diente = dienton[0]
        cpattern(1, False, ra, rf, z, diente, True, anchoeng, newComp, 'Join')
        combine(z, newComp)
        if vul2 == True:
            mirror(perfil[2], anchoeng, newComp)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
    # movebody(0,0,-(anchoeng)/2)
    # wormcut(m,z,radio)

def wormhelix2(m,z,ap,radio,fastcompute,cuerdas,cw,anchoeng, newComp):
    try:
        p=mt.pi*m
        radio=radio+1.25*m
        ah=mt.atan((mt.pi*m)/(mt.pi*2*radio))
        #if fastcompute==True:
        aok=50
        #else:
        #    aok=500
        tt=linspace(0,2*mt.pi,aok)
        bv=p/(2*mt.pi)
        coef=0
        if cw==True:
            coef=1
        zz=[]
        xx=[]
        yy=[]
        for i in range(0,len(tt)):
            zz.append(bv*tt[i])
            xx.append(radio*mt.cos(tt[i]+coef*mt.pi/2))
            yy.append(radio*mt.sin(tt[i]+coef*mt.pi/2))

        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        rootComp = newComp
        sketch5=rootComp.sketches.add(rootComp.xYConstructionPlane)
        sketch6=rootComp.sketches.add(rootComp.xYConstructionPlane)
        sketch3=rootComp.sketches.add(rootComp.xYConstructionPlane)
        orig=adsk.core.Point3D.create(0,0,0)
        extrudes=rootComp.features.extrudeFeatures

        helix = adsk.core.ObjectCollection.create()
        #if cw==True:
        #    for i in range (0,len(tt)):
        #        helix.add(adsk.core.Point3D.create(zz[i]/10,yy[i]/10,xx[i]/10))
        #elif cw==False:
        #    for i in range (0,len(tt)):
        #        helix.add(adsk.core.Point3D.create(zz[i]/10,xx[i]/10,yy[i]/10))
        #spline=sketch3.sketchCurves.sketchFittedSplines.add(helix)

        puntoai=adsk.core.Point3D.create(-((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap)))/10,(radio+m)/10,0)
        puntoad=adsk.core.Point3D.create(((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap)))/10,(radio+m)/10,0)
        puntoabi=adsk.core.Point3D.create(-((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap)))/10,(radio-1.30*m)/10,0)
        puntoabd=adsk.core.Point3D.create(((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap)))/10,(radio-1.30*m)/10,0)

        puntoai2=adsk.core.Point3D.create(-((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap)))/10+(1*mt.pi*m/2)/10,-(radio+m)/10,0)
        puntoad2=adsk.core.Point3D.create(((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap))+(1*mt.pi*m/2))/10,-(radio+m)/10,0)
        puntoabi2=adsk.core.Point3D.create(-((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap)))/10+(1*mt.pi*m/2)/10,-(radio-1.30*m)/10,0)
        puntoabd2=adsk.core.Point3D.create(((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap))+(1*mt.pi*m/2))/10,-(radio-1.30*m)/10,0)

        puntoai3=adsk.core.Point3D.create(-((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap)))/10+(2*mt.pi*m/2)/10,(radio+m)/10,0)
        puntoad3=adsk.core.Point3D.create(((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap))+(2*mt.pi*m/2))/10,(radio+m)/10,0)
        puntoabi3=adsk.core.Point3D.create(-((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap)))/10+(2*mt.pi*m/2)/10,(radio-1.30*m)/10,0)
        puntoabd3=adsk.core.Point3D.create(((mt.pi*m/4)+(1.30*m)/(mt.tan(mt.pi/2-ap))+(2*mt.pi*m/2))/10,(radio-1.30*m)/10,0)

        sketch5.sketchCurves.sketchLines.addByTwoPoints(puntoai,puntoad)
        sketch5.sketchCurves.sketchLines.addByTwoPoints(puntoai,puntoabi)
        sketch5.sketchCurves.sketchLines.addByTwoPoints(puntoabi,puntoabd)
        sketch5.sketchCurves.sketchLines.addByTwoPoints(puntoabd,puntoad)
        prof1=sketch5.profiles.item(0)

        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoai2, puntoad2)
        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoai2, puntoabi2)
        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoabi2, puntoabd2)
        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoabd2, puntoad2)

        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoai3, puntoad3)
        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoai3, puntoabi3)
        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoabi3, puntoabd3)
        sketch6.sketchCurves.sketchLines.addByTwoPoints(puntoabd3, puntoad3)
        prof2=sketch6.profiles.item(0)
        prof3=sketch6.profiles.item(1)
        sketch6.isVisible=False

        sketch6 = rootComp.sketches.add(rootComp.yZConstructionPlane)
        cirqulo = sketch6.sketchCurves.sketchCircles.addByCenterRadius(orig, (radio - 1.25 * m) / 10)


        puntoderecha=(((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap))+(1*mt.pi*m/2))/10)
        puntoizquierda=(-((mt.pi*m/4)-m/(mt.tan(mt.pi/2-ap)))/10+(1*mt.pi*m/2)/10)

        posx=(puntoizquierda+(puntoderecha-puntoizquierda)/2)/1
        circujo=(m*z/2)+m
        ladis=((m*z/2)+radio)
        puntote=adsk.core.Point3D.create(posx,-ladis/10,0)
        puntononon=adsk.core.Point3D.create(posx,-ladis/10,1)
        sketchuno=rootComp.sketches.add(rootComp.xYConstructionPlane)
        lineacentro=sketchuno.sketchCurves.sketchLines.addByTwoPoints(puntote,puntononon)
        sketchuno.sketchCurves.sketchCircles.addByCenterRadius(puntote,(circujo)/10)
        profuno=sketchuno.profiles.item(0)

        extinputuno = extrudes.createInput(profuno, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        extinputuno.setSymmetricExtent(adsk.core.ValueInput.createByReal((anchoeng)),True)
        extrudes.add(extinputuno)

        path=newComp.features.createPath(cirqulo,False)
        loft=rootComp.features.loftFeatures
        input=loft.createInput(adsk.fusion.FeatureOperations.CutFeatureOperation)
        loftobj = input.loftSections



        input2 = loft.createInput(adsk.fusion.FeatureOperations.CutFeatureOperation)
        loftobj2 = input2.loftSections
        if cw==False:
            loftobj.add(prof1)
            loftobj.add(prof2)
            loftobj2.add(prof2)
            loftobj2.add(prof3)
        elif cw==True:
            loftobj.add(prof2)
            loftobj.add(prof1)
            loftobj2.add(prof3)
            loftobj2.add(prof2)
        input.centerLineOrRails.addCenterLine(path)
        input2.centerLineOrRails.addCenterLine(path)

        corte1=loft.add(input)
        corte=loft.add(input2)

        sketch3.isVisible=False
        sketch5.isVisible = False
        sketch6.isVisible = False


        return corte,lineacentro,posx,ladis,corte1
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def indcpattern(feature,linecenter,z, newComp):
    a=feature
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = newComp
    eje = linecenter
    inputEntites = adsk.core.ObjectCollection.create()
    inputEntites.add(feature)
    circularFeats = rootComp.features.circularPatternFeatures
    circularFeatInput = circularFeats.createInput(inputEntites, eje)
    circularFeatInput.quantity = adsk.core.ValueInput.createByReal(z)
    circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    circularFeatInput.isSymmetric = False
    a = 2
    circularFeatInput.patternComputeOption = a
    circularFeat = circularFeats.add(circularFeatInput)

def cremsketch(m,z,h,ap,T,altura,anchoeng,ah,newComp):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = newComp
    # hacer un sketch
    sketch = rootComp.sketches.add(rootComp.xYConstructionPlane)
    orig = adsk.core.Point3D.create(0, 0, 0)
    extrudes = rootComp.features.extrudeFeatures
    sweeps = rootComp.features.sweepFeatures

    xaxis=rootComp.xConstructionAxis
    sketch2 = rootComp.sketches.add(rootComp.xYConstructionPlane)
    punto1 = adsk.core.Point3D.create(h * mt.tan(ap) / 10, h / 10, 0)
    lines = sketch.sketchCurves.sketchLines
    lines.addByTwoPoints(orig, punto1)
    puntillo = T - 2 * (m * mt.tan(ap))
    puntillo2 = T + 2 * (m * mt.tan(ap))
    punto2x = h * mt.tan(ap) + puntillo
    punto2 = adsk.core.Point3D.create(punto2x / 10, h / 10, 0)
    punto3 = adsk.core.Point3D.create((T + 2 * (1.25 * m * mt.tan(ap))) / 10, 0, 0)
    qq = (T + 2 * (1.25 * m * mt.tan(ap)))
    lines.addByTwoPoints(punto2, punto3)
    lines.addByTwoPoints(punto1, punto2)
    puntoab1 = adsk.core.Point3D.create(0, -altura, 0)
    lines.addByTwoPoints(orig, puntoab1)
    puntoab2 = adsk.core.Point3D.create((T + 2 * (1.25 * m * mt.tan(ap))) / 10, -altura, 0)
    puntoab22 = adsk.core.Point3D.create(qq / 10 + (mt.pi * m - puntillo2) / 10, -altura, 0)
    lines.addByTwoPoints(puntoab1, puntoab2)
    punto4 = adsk.core.Point3D.create(abs(qq / 10 + (mt.pi * m - puntillo2)) / 10, 0, 0)
    lines.addByTwoPoints(puntoab2, punto3)
    linea = lines.addByTwoPoints(punto4, adsk.core.Point3D.create(10, 0, 0))
    prof = sketch.profiles.item(0)


    punton=adsk.core.Point3D.create((anchoeng*mt.sin(ah)),0,anchoeng)
    pat=lines.addByTwoPoints(orig,punton)
    path=newComp.features.createPath(pat,False)

    lines2 = sketch2.sketchCurves.sketchLines
    dtotal = (z - 1) * mt.pi * m + T + 2.5 * m * mt.tan(ap)+(anchoeng*mt.sin(ah))*10
    dtorig= (z - 1) * mt.pi * m + T + 2.5 * m * mt.tan(ap)
    cantidad=mt.floor((dtotal-dtorig)/((T + 2 * (1.25 * m * mt.tan(ap)))+mt.pi*m-(T + 2 * (1.25 * m * mt.tan(ap)))))

    pt2 = adsk.core.Point3D.create(dtotal / 10, 0, 0)
    lines2.addByTwoPoints(orig, pt2)
    lines2.addByTwoPoints(orig, puntoab1)
    pt3 = adsk.core.Point3D.create(dtotal / 10, -altura, 0)
    lines2.addByTwoPoints(puntoab1, pt3)
    lines2.addByTwoPoints(pt3, pt2)
    prof2 = sketch2.profiles.item(0)
    plinea=newComp.features.createPath(linea,False)

    plineaneg=0
    prof3=0
    prof32=0
    if cantidad>0:
        sketch3=rootComp.sketches.add(rootComp.xYConstructionPlane)
        lineas=sketch3.sketchCurves.sketchLines

        puntocortearri=adsk.core.Point3D.create(dtotal/10,2.25*m/10,0)
        puntocortearrd=adsk.core.Point3D.create(dtotal/10+(cantidad*T + 2 * (1.25 * m * mt.tan(ap))+mt.pi*m+(anchoeng*mt.sin(ap))*10)/10,2.25*m/10,0)
        puntocorteab=adsk.core.Point3D.create(dtotal/10+(cantidad*T + 2 * (1.25 * m * mt.tan(ap))+mt.pi*m+(anchoeng*mt.sin(ap))*10)/10,-altura,0)
        lineas.addByTwoPoints(puntocortearri,puntocortearrd)
        lineas.addByTwoPoints(puntocorteab,puntocortearrd)
        lineas.addByTwoPoints(pt3,puntocorteab)
        lineas.addByTwoPoints(pt3,puntocortearri)
        prof3=sketch3.profiles.item(0)

        #aqui va el perfil de corte de la izquierda
        par=adsk.core.Point3D.create(0,2.25*m/10,0)
        pab=adsk.core.Point3D.create(0,-altura,0)
        paiz=adsk.core.Point3D.create(-(cantidad*T + 2 * (1.25 * m * mt.tan(ap))+mt.pi*m+(anchoeng*mt.sin(ap))*10)/10,2.25*m/10,0)
        pabiz=adsk.core.Point3D.create(-(cantidad*T + 2 * (1.25 * m * mt.tan(ap))+mt.pi*m+(anchoeng*mt.sin(ap))*10)/10,-altura,0)
        lineas.addByTwoPoints(paiz,par)
        lineas.addByTwoPoints(paiz, pabiz)
        lineas.addByTwoPoints(par, pab)
        lineas.addByTwoPoints(pab, pabiz)
        prof32=sketch3.profiles.item(1)

        #crea el path negativo
        lineaneg = lines.addByTwoPoints(punto4, adsk.core.Point3D.create(-1, 0, 0))
        lineaneg.isConstruction=True
        plineaneg = newComp.features.createPath(lineaneg, False)




    return prof,path,prof2,plinea,cantidad,plineaneg,prof3,prof32

def simplesweep(prof,path,newComp):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = newComp
    try:
        sweeps=rootComp.features.sweepFeatures
        sweepInput=sweeps.createInput(prof, path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        sweepInput.path=path
        swep=sweeps.add(sweepInput)
        return swep
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def cremspatt(body,path,z,paso,newComp):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = newComp
    pathpattern = rootComp.features.pathPatternFeatures
    inputentites = adsk.core.ObjectCollection.create()
    inputentites.add(body)
    quantity = adsk.core.ValueInput.createByReal(z)
    p = adsk.core.ValueInput.createByReal(paso)

    patternInput = pathpattern.createInput(inputentites, path, quantity, p, 1)
    pathpattern.add(patternInput)

def fichatecnica(Fc,escorona,eshelicoidal,esconico,esgusano,esPS,modulo,ap,z,ah,espesorc,z2,radiotornillo,X,newComp):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = newComp
    # hacer un sketch
    cadena="FC=" + str(Fc) + " m=" + str(modulo) + " PA=" + str(radToDeg(ap)) + "°" + " z=" + str(z)
    if escorona==True and eshelicoidal==False:
        cadena=cadena + "\n" + "Radial Thickness=" + str(espesorc) + "mm"
    if eshelicoidal==True and escorona==False:
        cadena=cadena + " HA=" + str(round(radToDeg(ah),1)) + "°"
    if eshelicoidal == True and escorona == True:
        cadena = cadena  + " HA=" + str(round(radToDeg(ah),1)) + "°" + "\n" + "Radial Thickness=" + str(espesorc) + "mm"
    if esconico==True:
        cadena=cadena + "\n" + "z2=" + str(z2)
    if esgusano==True:
        cadena=cadena + "\n" + " Screw Radius=" + str(radiotornillo) + "mm"
    if esPS==True and eshelicoidal==False:
        cadena= cadena + "\n" + "X=" + str(X)
    if esPS==True and eshelicoidal==True:
        cadena = cadena + " HA=" + str(round(radToDeg(ah),1)) + "°" + "\n" + "X=" + str(X)
    sketch = rootComp.sketches.add(rootComp.xYConstructionPlane)
    sketchTexts = sketch.sketchTexts
    oint = adsk.core.Point3D.create(0, 0,0)
    sketchTextInput = sketchTexts.createInput(cadena, .30, oint)
    sketchTextInput.textStyle = adsk.fusion.TextStyles.TextStyleBold
    sketchText = sketchTexts.add(sketchTextInput)
    sketch.isVisible=False

command_cache = {}
def get(obj, key, default=None):
    if type(obj) in command_cache and key in command_cache[type(obj)]:
        return command_cache[type(obj)][key]
    return default 
def set(obj, key, value):
    global command_cache
    if type(obj) not in command_cache:
        command_cache[type(obj)] = {}
    command_cache[type(obj)][key] = value

class cmdDefPressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app=adsk.core.Application.get()
        ui=app.userInterface
        cmd=args.command
        inputs=cmd.commandInputs
        ValueInput1=adsk.core.ValueInput.createByReal(1)

        # Fusion's default units are cm, since your units are mm you'll need to divide that value with 10
        # a ValueInput = 1 will show as 10mm
        aaok=inputs.addBoolValueInput('aok1','Fast Compute',True,'',defaultfc)
        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(.03))
        # u=inputs.addDropDownCommandInput('DropDownCommandInput1','Module [mm]',1)
        # qty=u.listItems
        # qty.add('0.3 mm',True,'si')
        # for nn in range(0,len(list)):
        #     qty.add(list[nn],False)
        inputs.addIntegerSpinnerCommandInput('IntegerSpinner1', 'Number of teeth [ ]', 6, 250, 1, 17)
        inputs.addValueInput('ValueInput1', 'Gear height [mm]', 'mm', ValueInput1)
        inputs.addFloatSpinnerCommandInput('FloatSpinner1', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, 14.5)
        # con esto vinculo al boton OK
        onExecute=cmdDefOKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)

class cmdDef2PressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        cmd = args.command
        inputs = cmd.commandInputs

        ValueInput2=adsk.core.ValueInput.createByReal(1)
        ValueInput22=adsk.core.ValueInput.createByReal(.5)

        aaok2=inputs.addBoolValueInput('aok2', 'Fast Compute', True, '', defaultfc)
        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(.03))
        # u=inputs.addDropDownCommandInput('DropDownCommandInput2','Module [mm]',1)
        # qty=u.listItems
        # qty.add('0.3 mm',True,'si')
        # for nn in range(0,len(list)):
        #     qty.add(list[nn],False)
        inputs.addIntegerSpinnerCommandInput('IntegerSpinner2', 'Number of teeth [ ]', 6, 250, 1, 17)
        inputs.addValueInput('ValueInput2', 'Gear height [mm]', 'mm', ValueInput2)
        inputs.addFloatSpinnerCommandInput('FloatSpinner2', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, 14.5)
        inputs.addValueInput('ValueInput22','Radial thickness [mm]','mm',ValueInput22)
        # con esto vinculo al boton OK
        onExecute=cmdDef2OKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)

class cmdDef3PressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        cmd = args.command
        inputs = cmd.commandInputs

        ValueInput3=adsk.core.ValueInput.createByReal(1)
        ValueInput32=adsk.core.ValueInput.createByReal(0.5)

        aaok3=inputs.addBoolValueInput('aok3', 'Fast Compute', True, '', defaultfc)
        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(.03))
        # u=inputs.addDropDownCommandInput('DropDownCommandInput3','Module [mm]',1)
        # qty=u.listItems
        # qty.add('0.3 mm',True,'si')
        # for nn in range(0,len(list)):
        #     qty.add(list[nn],False)
        inputs.addIntegerSpinnerCommandInput('IntegerSpinner3', 'Number of teeth [ ]', 6, 250, 1, 17)
        inputs.addValueInput('ValueInput3', 'Gear height [mm]', 'mm', ValueInput3)
        inputs.addFloatSpinnerCommandInput('FloatSpinner3', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, 14.5)
        inputs.addValueInput('ValueInput32','Radial thickness [mm]','mm',ValueInput32)

        # con esto vinculo al boton OK
        onExecute=cmdDef3OKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)

class cmdDef4PressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app=adsk.core.Application.get()
        ui=app.userInterface
        cmd=args.command
        inputs=cmd.commandInputs

        ValueInput4=adsk.core.ValueInput.createByReal(1)

        aaok4=inputs.addBoolValueInput('aok4', 'Fast Compute', True, '', defaultfc)
        inputs.addBoolValueInput('BoolValue4', 'Clock Wise', True, '', False)
        inputs.addBoolValueInput('BoolValue42','Double Helical',True,'',False)
        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(.03))
        # u=inputs.addDropDownCommandInput('DropDownCommandInput4','Module [mm]',1)
        # qty=u.listItems
        # qty.add('0.3 mm',True,'si')
        # for nn in range(0,len(list)):
        #     qty.add(list[nn],False)
        inputs.addIntegerSpinnerCommandInput('IntegerSpinner4', 'Number of teeth [ ]', 6, 250, 1, 17)
        inputs.addValueInput('ValueInput4', 'Gear height [mm]', 'mm', ValueInput4)
        inputs.addFloatSpinnerCommandInput('FloatSpinner4', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, 14.5)
        inputs.addFloatSpinnerCommandInput('FloatSpinner42','Helix angle [°]','deg',0,45,0.5,15)
        # con esto vinculo al boton OK

        onExecute=cmdDef4OKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)

class cmdDef5PressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app=adsk.core.Application.get()
        ui=app.userInterface
        cmd=args.command
        inputs=cmd.commandInputs

        ValueInput5=adsk.core.ValueInput.createByReal(1)
        ValueInput52=adsk.core.ValueInput.createByReal(.5)

        aaok5=inputs.addBoolValueInput('aok5', 'Fast Compute', True, '', defaultfc)
        inputs.addBoolValueInput('BoolValue5', 'Clock Wise', True, '', False)
        inputs.addBoolValueInput('BoolValue52','Double Helical',True,'',False)
        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(.03))
        # u=inputs.addDropDownCommandInput('DropDownCommandInput5','Module [mm]',1)
        # qty=u.listItems
        # qty.add('0.3 mm',True,'si')
        # for nn in range(0,len(list)):
        #     qty.add(list[nn],False)
        inputs.addIntegerSpinnerCommandInput('IntegerSpinner5', 'Number of teeth [ ]', 6, 250, 1, 17)
        inputs.addValueInput('ValueInput5', 'Gear height [mm]', 'mm', ValueInput5)
        inputs.addFloatSpinnerCommandInput('FloatSpinner5', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, 14.5)
        inputs.addValueInput('ValueInput52','Radial thickness [mm]','mm',ValueInput52)
        inputs.addFloatSpinnerCommandInput('FloatSpinner52', 'Helix angle [°]', 'deg', 0, 45, 0.5, 15)

        #con esto vinculo al boton OK
        onExecute=cmdDef5OKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)

class cmdDef6PressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app=adsk.core.Application.get()
        ui=app.userInterface
        cmd=args.command
        inputs=cmd.commandInputs

        ValueInput6=adsk.core.ValueInput.createByReal(1)
        ValueInput62=adsk.core.ValueInput.createByReal(.5)

        aaok6=inputs.addBoolValueInput('aok6', 'Fast Compute', True, '', defaultfc)
        inputs.addBoolValueInput('BoolValue6', 'Clock Wise', True, '', False)
        inputs.addBoolValueInput('BoolValue62','Double Helical',True,'',False)
        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(.03))
        # u=inputs.addDropDownCommandInput('DropDownCommandInput6','Module [mm]',1)
        # qty=u.listItems
        # qty.add('0.3 mm',True,'si')
        # for nn in range(0,len(list)):
        #     qty.add(list[nn],False)
        inputs.addIntegerSpinnerCommandInput('IntegerSpinner6', 'Number of teeth [ ]', 6, 250, 1, 17)
        inputs.addValueInput('ValueInput6', 'Gear height [mm]', 'mm', ValueInput6)
        inputs.addFloatSpinnerCommandInput('FloatSpinner6', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, 14.5)
        inputs.addValueInput('ValueInput62','Radial thickness [mm]','mm',ValueInput62)
        inputs.addFloatSpinnerCommandInput('FloatSpinner62', 'Helix angle [°]', 'deg', 0, 45, 0.5, 15)

        #con esto vinculo al boton OK
        onExecute=cmdDef6OKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)

class cmdDef7PressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app=adsk.core.Application.get()
        ui=app.userInterface
        cmd=args.command
        inputs=cmd.commandInputs
        ValueInput7 = adsk.core.ValueInput.createByReal(1)

        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(.03))
        # u = inputs.addDropDownCommandInput('DropDownCommandInput7', 'Module [mm]', 1)
        # qty = u.listItems
        # qty.add('0.3 mm', True, 'si')
        # for nn in range(0, len(list)):
        #     qty.add(list[nn], False)
        inputs.addIntegerSpinnerCommandInput('IntegerSpinner7', 'Number of teeth [ ]', 6, 250, 1, 17)
        inputs.addValueInput('ValueInput7', 'Rack thickness [mm]', 'mm', ValueInput7)
        inputs.addFloatSpinnerCommandInput('FloatSpinner7', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, 14.5)
        inputs.addFloatSpinnerCommandInput('FloatSpinner72', 'Helix angle [°]', 'deg', 0, 45, 0.5, 0)
        inputs.addValueInput('ValueInput72', 'Rack height [mm]', 'mm', ValueInput7)
        # con esto vinculo al boton OK
        onExecute = cmdDef7OKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)

class cmdDef8PressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app=adsk.core.Application.get()
        ui=app.userInterface
        cmd=args.command
        inputs=cmd.commandInputs
        ValueInput8=adsk.core.ValueInput.createByReal(1)

        aaok8=inputs.addBoolValueInput('aok8', 'Fast Compute', True, '', defaultfc)
        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(.03))
        # u=inputs.addDropDownCommandInput('DropDownCommandInput8','Module [mm]',1)
        # qty=u.listItems
        # qty.add('0.3 mm',True,'si')
        # for nn in range(0,len(list)):
        #     qty.add(list[nn],False)
        inputs.addIntegerSpinnerCommandInput('IntegerSpinner8', 'Number of teeth for the wheel [ ]', 6, 250, 1, 17)
        inputs.addIntegerSpinnerCommandInput('IntegerSpinner82', 'Number of teeth for the pinion [ ]', 6, 250, 1, 17)
        inputs.addFloatSpinnerCommandInput('FloatSpinner8', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, 14.5)
        # con esto vinculo al boton OK
        onExecute=cmdDef8OKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)

class cmdDef9PressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app=adsk.core.Application.get()
        ui=app.userInterface
        cmd=args.command
        inputs=cmd.commandInputs
        ValueInput9=adsk.core.ValueInput.createByReal(1)

        aaok9=inputs.addBoolValueInput('aok9','Fast Compute', True, '', defaultfc)
        inputs.addFloatSpinnerCommandInput('FloatSpinner92','Profile shifting coefficient "X" [ ]','',-1,1,.01,0)
        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(.03))
        # u=inputs.addDropDownCommandInput('DropDownCommandInput9','Module [mm]',1)
        # qty=u.listItems
        # qty.add('0.3 mm',True,'si')
        # for nn in range(0,len(list)):
        #     qty.add(list[nn],False)
        inputs.addIntegerSpinnerCommandInput('IntegerSpinner9', 'Number of teeth [ ]', 6, 250, 1, 17)
        inputs.addValueInput('ValueInput9', 'Gear height [mm]', 'mm', ValueInput9)
        inputs.addFloatSpinnerCommandInput('FloatSpinner9', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, 14.5)
        # con esto vinculo al boton OK
        onExecute=cmdDef9OKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)

class cmdDef10PressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app=adsk.core.Application.get()
        ui=app.userInterface
        cmd=args.command
        inputs=cmd.commandInputs

        ValueInput10=adsk.core.ValueInput.createByReal(1)

        aaok10=inputs.addBoolValueInput('aok10', 'Fast Compute', True, '', defaultfc)
        inputs.addBoolValueInput('BoolValue10', 'Clock Wise', True, '', False)
        inputs.addBoolValueInput('BoolValue102','Double Helical',True,'',False)
        inputs.addFloatSpinnerCommandInput('FloatSpinner103','Profile shifting coefficient "X" [ ]','',-1,1,.01,0)
        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(.03))
        # u=inputs.addDropDownCommandInput('DropDownCommandInput10','Module [mm]',1)
        # qty=u.listItems
        # qty.add('0.3 mm',True,'si')
        # for nn in range(0,len(list)):
        #     qty.add(list[nn],False)
        inputs.addIntegerSpinnerCommandInput('IntegerSpinner10', 'Number of teeth [ ]', 6, 250, 1, 17)
        inputs.addValueInput('ValueInput10', 'Gear height [mm]', 'mm', ValueInput10)
        inputs.addFloatSpinnerCommandInput('FloatSpinner10', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, 14.5)
        inputs.addFloatSpinnerCommandInput('FloatSpinner102','Helix angle [°]','deg',0,45,0.5,15)
        # con esto vinculo al boton OK

        onExecute=cmdDef10OKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)

class cmdDef11PressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app=adsk.core.Application.get()
        ui=app.userInterface
        cmd=args.command
        inputs=cmd.commandInputs

        ValueInput11=adsk.core.ValueInput.createByReal(1)
        ValueInput112= adsk.core.ValueInput.createByReal(1)
        ValueInput113 = adsk.core.ValueInput.createByReal(.5)

        inputs.addBoolValueInput('aok11', 'Fast Compute', True, '', defaultfc)
        brow11=inputs.addButtonRowCommandInput('Brow11','Worm Gear Type',False)
        brow11.listItems.add('Helical',True,'Resources/Helical')
        brow11.listItems.add('Hobbed Straight',False,'Resources/HobbedWorm')

        #usar clock wise como rosca izquierda o derecha
        inputs.addBoolValueInput('BoolValue11', 'Left threaded', True, '', False)
        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(.03))
        # u=inputs.addDropDownCommandInput('DropDownCommandInput11','Module [mm]',1)
        # qty=u.listItems
        # qty.add('0.3 mm',True,'si')
        # for nn in range(0,len(list)):
        #     qty.add(list[nn],False)
        inputs.addIntegerSpinnerCommandInput('IntegerSpinner11', 'Number of teeth [ ]', 6, 250, 1, 17)
        inputs.addValueInput('IntegerSpinner112','Worm length [mm]','mm',ValueInput112)
        inputs.addValueInput('ValueInput11', 'Worm gear height [mm]', 'mm', ValueInput11)
        inputs.addFloatSpinnerCommandInput('FloatSpinner11', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, 14.5)
        inputs.addValueInput('FloatSpinner112','Worm drive radius [mm]','mm',ValueInput113 )
        # con esto vinculo al boton OK

        onExecute=cmdDef11OKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)

#Aquí van los Botones de OK
class cmdDefOKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        try:
            eventArgs=adsk.core.CommandEventArgs.cast(args)
            app=adsk.core.Application.get()
            ui=app.userInterface
            design = app.activeProduct
            inputs2=eventArgs.command.commandInputs
            #Recopila los valores introducidos por el usuario notese que 'a' es un string y debe eliminar la parte de mm para convertirlo a float
            aaok=inputs2.itemById('aok1').value
            z=inputs2.itemById('IntegerSpinner1').value
            #q=inputs2.itemById('DropDownCommandInput1').selectedItem.name
            anchoeng=inputs2.itemById('ValueInput1').value
            # Fusion's default units are cm, since you're using mm you'll have to multiply the value per 20
            m=inputs2.itemById('Module').value*10
            # a=str(q[0:len(q)-3])
            # m=float(a)
            # ui.messageBox(str(m) + " " +str(juan))
            ap=inputs2.itemById('FloatSpinner1').value
            
            design = adsk.fusion.Design.cast(app.activeProduct)
            root = design.activeComponent
            newComp = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
            hb=hidebodies(newComp)
            planetgearsdr(m,z,ap,aaok,anchoeng,newComp)
            showhiddenbodies(hb, newComp)
            fichatecnica(aaok,False,False,False,False,False, m,ap,z,0,0,0,0,0,newComp)
            htl(8)
            # nummerop=5


        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#Ejecución para crear un engrane DR

class cmdDef2OKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        eventArgs=adsk.core.CommandEventArgs.cast(args)
        app=adsk.core.Application.get()
        ui=app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.activeComponent
        newComp = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
        inputs2=eventArgs.command.commandInputs

        aaok=inputs2.itemById('aok2').value
        z=inputs2.itemById('IntegerSpinner2').value
        #q=inputs2.itemById('DropDownCommandInput2').selectedItem.name
        anchoeng=inputs2.itemById('ValueInput2').value
        m=inputs2.itemById('Module').value*10
        # a=str(q[0:len(q) - 3])
        # m=float(a)
        ap=inputs2.itemById('FloatSpinner2').value
        espesorc=inputs2.itemById('ValueInput22').value*10
        hb=hidebodies(newComp)
        coronastd(aaok,z,m,ap,anchoeng,espesorc,newComp)
        showhiddenbodies(hb,newComp)
        fichatecnica(aaok,True,False,False,False,False,m,ap,z,0,espesorc,0,0,0,newComp)
        #numerop=5
        htl(7)

#Botón de ejecución para las coronas estándar DR

class cmdDef3OKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        eventArgs=adsk.core.CommandEventArgs.cast(args)
        app=adsk.core.Application.get()
        ui=app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.activeComponent
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        newComp = occ.component
        inputs2=eventArgs.command.commandInputs
        try:
            aaok=inputs2.itemById('aok3').value
            z=inputs2.itemById('IntegerSpinner3').value
            #q=inputs2.itemById('DropDownCommandInput3').selectedItem.name
            anchoeng=inputs2.itemById('ValueInput3').value
            m=inputs2.itemById('Module').value*10
            # a=str(q[0:len(q) - 3])
            # m=float(a)
            ap=inputs2.itemById('FloatSpinner3').value
            espesorc=inputs2.itemById('ValueInput32').value*10
            hb=hidebodies(newComp)
            coronasnostd(aaok,z,m,anchoeng,ap,espesorc, newComp, occ)
            showhiddenbodies(hb,newComp)
            #numerop=7
            if (m*z/2+m + espesorc) < (m*z/2-1.25*m + m*z/2+m):
                fichatecnica(aaok, True, False, False, False, False, m, ap, z, 0, espesorc, 0, 0, 0,newComp)
                htl(11)
            else:
                fichatecnica(aaok, True, False, False, False, False, m, ap, z, 0, espesorc, 0, 0, 0,newComp)
                htl(9)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#Ejecución para la creación de una corona DR no estándar

class cmdDef4OKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        try:
            eventArgs=adsk.core.CommandEventArgs.cast(args)
            app=adsk.core.Application.get()
            ui=app.userInterface
            design=app.activeProduct
            rootComp=design.rootComponent
            inputs2=eventArgs.command.commandInputs

            aaok=inputs2.itemById('aok4').value
            vul=inputs2.itemById('BoolValue4').value
            vul2=inputs2.itemById('BoolValue42').value
            mult1=1
            if vul2==True:
                mult1=2
            z=inputs2.itemById('IntegerSpinner4').value
            # q=inputs2.itemById('DropDownCommandInput4').selectedItem.name
            anchoeng=inputs2.itemById('ValueInput4').value/mult1
            # a=str(q[0:len(q) - 3])
            # m=float(a)
            m=inputs2.itemById('Module').value*10
            ap=inputs2.itemById('FloatSpinner4').value
            ah=inputs2.itemById('FloatSpinner42').value

            design = adsk.fusion.Design.cast(app.activeProduct)
            root = design.activeComponent
            newComp = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component

            hb=hidebodies(newComp)
            helicalgs(aaok,vul,vul2,z,anchoeng,m,ap,ah,newComp)
            showhiddenbodies(hb,newComp)
            #numerop hsimple=5
            if vul2==True:
                fichatecnica(aaok,False,True,False,False,False, m,ap,z,ah,0,0,0,0,newComp)
                htl(11)
            else:
                fichatecnica(aaok, False, True, False, False, False, m, ap, z, ah, 0, 0, 0, 0,newComp)
                htl(8)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
        #numerop doble=8

#Ejecución para la creación de un engrane helicoidal

class cmdDef5OKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        eventArgs=adsk.core.CommandEventArgs.cast(args)
        app=adsk.core.Application.get()
        ui=app.userInterface
        inputs2=eventArgs.command.commandInputs
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.activeComponent
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        newComp = occ.component

        aaok=inputs2.itemById('aok5').value
        vul=inputs2.itemById('BoolValue5').value
        vul2=inputs2.itemById('BoolValue52').value
        mult1=1
        if vul2==True:
            mult1 =2
        z=inputs2.itemById('IntegerSpinner5').value
        # q=inputs2.itemById('DropDownCommandInput5').selectedItem.name
        anchoeng=inputs2.itemById('ValueInput5').value / mult1
        m=inputs2.itemById('Module').value*10
        # a=str(q[0:len(q)-3])
        # m=float(a)
        ap=inputs2.itemById('FloatSpinner5').value
        espesorc=inputs2.itemById('ValueInput52').value*10
        ah=inputs2.itemById('FloatSpinner52').value
        hb=hidebodies(newComp)
        try:
            coronashelnostdr(aaok,vul,vul2,z,anchoeng,m,ap,espesorc,ah, newComp, occ)
            #numeropsimple= 8
            #numerop doble=11
            if (m*z/2+m + espesorc) < (m*z/2-1.25*m + m*z/2+m):
                num=2
            else:
                num=0
            if vul2 == True:
                fichatecnica(aaok,True,True,False,False,False,m,ap,z,ah,espesorc,0,0,0,newComp)
                htl(12+num)
            else:
                fichatecnica(aaok, True, True, False, False, False, m, ap, z, ah, espesorc, 0, 0, 0, newComp)
                htl(9+num)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
        showhiddenbodies(hb, newComp)
#Ejecución para la creación de una corona heicoidal no stdr


class cmdDef6OKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        eventArgs=adsk.core.CommandEventArgs.cast(args)
        app=adsk.core.Application.get()
        ui=app.userInterface
        inputs2=eventArgs.command.commandInputs
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.activeComponent
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        newComp = occ.component
        try:
            aaok=inputs2.itemById('aok6').value
            vul=inputs2.itemById('BoolValue6').value
            vul2=inputs2.itemById('BoolValue62').value
            mult1=1
            if vul2==True:
                mult1=2
            z=inputs2.itemById('IntegerSpinner6').value
            # q=inputs2.itemById('DropDownCommandInput6').selectedItem.name
            anchoeng=inputs2.itemById('ValueInput6').value/mult1
            m=inputs2.itemById('Module').value*10
            # a=str(q[0:len(q)-3])
            # m=float(a)
            ap=inputs2.itemById('FloatSpinner6').value
            espesorc=inputs2.itemById('ValueInput62').value*10
            ah=inputs2.itemById('FloatSpinner62').value
            hb=hidebodies(newComp)
            try:
                coronashelstdr(aaok,vul,vul2,z,anchoeng,m,ap,espesorc,ah, newComp)
                #numerop simple=5
                #numerop doble=8
                if vul2 == True:
                    fichatecnica(aaok, True, True, False, False, False, m, ap, z, ah, espesorc, 0, 0, 0, newComp)
                    htl(10)
                else:
                    fichatecnica(aaok, True, True, False, False, False, m, ap, z, ah, espesorc, 0, 0, 0, newComp)
                    htl(7)
            except:
                if ui:
                    ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
            showhiddenbodies(hb, newComp)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

#Ejecución para la creación de una corona Helicoidal stdr

class cmdDef7OKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        eventArgs=adsk.core.CommandEventArgs.cast(args)
        app=adsk.core.Application.get()
        ui=app.userInterface
        design = app.activeProduct
        rootComp = design.rootComponent
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.activeComponent
        newComp = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
        rootComp = newComp
        
        inputs2 = eventArgs.command.commandInputs
#Recopila los valores introducidos por el usuario notese que 'a' es un string y debe eliminar la parte de mm para convertirlo a float
        z = inputs2.itemById('IntegerSpinner7').value
        #q = inputs2.itemById('DropDownCommandInput7').selectedItem.name
        anchoeng = inputs2.itemById('ValueInput7').value
        m=inputs2.itemById('Module').value*10
        #a = str(q[0:len(q) - 3])
        #m = float(a)
        ap = inputs2.itemById('FloatSpinner7').value
        ah = inputs2.itemById('FloatSpinner72').value
        altura = inputs2.itemById('ValueInput72').value
        T = mt.pi * m / 2
        h = 2.25 * m
        pitch = (mt.pi * m / 10)
        try:
            cuentaprevios=rootComp.sketches.count
            hb2 = hidebodies(newComp)
            quantity = adsk.core.ValueInput.createByReal(z + 2)
            paso = adsk.core.ValueInput.createByReal(mt.pi * m / 10)
            crema = cremsketch(m, z, h, ap, T, altura, anchoeng, ah,newComp)
            if ah==0:
                esque = rootComp.sketches.item(rootComp.sketches.count - 2)
            else:
                if rootComp.sketches.count <3+cuentaprevios:
                    esque = rootComp.sketches.item(rootComp.sketches.count - 2)
                else:
                    esque = rootComp.sketches.item(rootComp.sketches.count - 3)

            profe = esque.profiles.item(0)
            diente = simplesweep(profe, crema[1], newComp)
            cremspatt(diente, crema[3], z + crema[4], pitch, newComp)
            if crema[4] > 0:
                cremspatt(diente, crema[5], crema[4] + 1, pitch, newComp)
            extruir(crema[2], anchoeng, newComp, 'Join')
            if crema[4] > 0:
                esque = rootComp.sketches.item(rootComp.sketches.count - 1)
                profe1=esque.profiles.item(0)
                profe2=esque.profiles.item(1)
                extruir(profe1, anchoeng, newComp, 'Cut')
                extruir(profe2, anchoeng, newComp, 'Cut')
            # roto la cremallera
            conta = rootComp.bRepBodies.count
            vec = adsk.core.Vector3D.create(1, 0, 0)
            objc = adsk.core.ObjectCollection.create()
            objc.add(rootComp.bRepBodies.item(conta - 1))
            transform = adsk.core.Matrix3D.create()
            transform.setToRotation(mt.pi / 2, vec, adsk.core.Point3D.create(0, -altura, 0))
            movefeats = rootComp.features.moveFeatures
            movefeatureInput = movefeats.createInput(objc, transform)
            movefeats.add(movefeatureInput)
            if ah!=0:
                eshelicoidal=True
            else:
                eshelicoidal = False
            if crema[4]>0:
                fichatecnica(True,False,eshelicoidal,False,False,False,m,ap,z,ah,0,0,0,0,newComp)
                htl(12)
            else:
                fichatecnica(True, False, eshelicoidal, False, False, False, m, ap, z, ah, 0, 0, 0, 0,newComp)
                htl(8)
            showhiddenbodies(hb2,newComp)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#Ejecución de creación de cremallera

class cmdDef8OKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.activeComponent
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        newComp = occ.component
        hb=hidebodies(newComp)
        try:
            
            design = app.activeProduct
            rootComp = newComp
            eventArgs=adsk.core.CommandEventArgs.cast(args)
            app=adsk.core.Application.get()
            ui=app.userInterface
            #design = app.activeProduct
            inputs2=eventArgs.command.commandInputs
            #Recopila los valores introducidos por el usuario notese que 'a' es un string y debe eliminar la parte de mm para convertirlo a float
            aaok=inputs2.itemById('aok8').value
            z=inputs2.itemById('IntegerSpinner8').value
            z2=inputs2.itemById('IntegerSpinner82').value
            m=inputs2.itemById('Module').value*10
            # q=inputs2.itemById('DropDownCommandInput8').selectedItem.name
            # a=str(q[0:len(q)-3])
            # m=float(a)
            ap=inputs2.itemById('FloatSpinner8').value
            list3=parameters(m,z,ap,0,1,False,0,aaok)
            rf=list3[0]
            x=list3[1]
            y=list3[2]
            x2=list3[3]
            y2=list3[4]
            aok=list3[5]
            Ttda=list3[6]
            ra=list3[7]
            rp=m*z/2
            rp2=m*z2/2
            aconico=mt.atan(z/z2)
            s=sketchcon(x,y,x2,y2,z,z2,rp,rp2,rf,ra,Ttda,m,aok, newComp)
            esq = rootComp.sketches.item(rootComp.sketches.count - 3)
            prof = esq.profiles.item(0)
            diente=sweep(prof,s[1],s[2],s[3],s[4], newComp)
            rev(s[5],s[6], newComp, 'NewBody')
            cpattern(s[6],True,ra,rf,z,diente,True,1, newComp, 'Join')
            combine(z, newComp)
            rev(s[7], s[6], newComp, 'Cut')
            rev(s[8], s[6], newComp,'Cut')
            rotcon(rf,aconico, occ)

            list4=parameters(m,z2,ap,0,1,False,0,aaok)
            rf4=list4[0]
            x4=list4[1]
            y4=list4[2]
            x24=list4[3]
            y24=list4[4]
            aok4=list4[5]
            Ttda4=list4[6]
            ra4=list4[7]
            rp4=m*z/2

            moveocc((ra4+5+ra)/10,0,0, occ)

            occ2 = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            newComp2 = occ2.component

            rp24=m*z2/2
            aconico4=mt.atan(z2/z)
            s4=sketchcon(x4,y4,x24,y24,z2,z,rp24,rp4,rf4,ra4,Ttda4,m,aok4, newComp2)
            esq = newComp2.sketches.item(newComp2.sketches.count - 3)
            prof = esq.profiles.item(0)
            diente=sweep(prof,s4[1],s4[2],s4[3],s4[4], newComp2)
            rev(s4[5],s4[6], newComp2,'NewBody')
            cpattern(s4[6],True,ra4,rf4,z2,diente,True,1, newComp2,'Join')
            combine(z2, newComp2)
            rev(s4[7],s4[6], newComp2,'Cut')
            rev(s4[8],s4[6], newComp2, 'Cut')
            rotcon(rf4,aconico4, occ2)
            fichatecnica(aaok,False,False,True,False,False,m,ap,z,0,0,z2,0,0, newComp2)
            htl(21)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
        showhiddenbodies(hb, newComp)
#Ejecución de creación de cónicos


class cmdDef9OKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        try:
            eventArgs=adsk.core.CommandEventArgs.cast(args)
            app=adsk.core.Application.get()
            ui=app.userInterface
            design = adsk.fusion.Design.cast(app.activeProduct)
            root = design.activeComponent
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            newComp = occ.component
            inputs2=eventArgs.command.commandInputs
            #Recopila los valores introducidos por el usuario notese que 'a' es un string y debe eliminar la parte de mm para convertirlo a float
            aaok=inputs2.itemById('aok9').value
            z=inputs2.itemById('IntegerSpinner9').value
            # q=inputs2.itemById('DropDownCommandInput9').selectedItem.name
            anchoeng=inputs2.itemById('ValueInput9').value
            m=inputs2.itemById('Module').value*10
            # a=str(q[0:len(q)-3])
            # m=float(a)
            ap=inputs2.itemById('FloatSpinner9').value
            X=inputs2.itemById('FloatSpinner92').value
            if abs(X)==0:
                X=0
            hb=hidebodies(newComp)
            list3=parameters(m,z,ap,0,anchoeng,False,X,aaok)
            rf=list3[0]
            x=list3[1]
            y=list3[2]
            x2=list3[3]
            y2=list3[4]
            aok=list3[5]
            Ttda=list3[6]
            ra=list3[7]
            rva=list3[28]
            rvf=list3[29]
            rb=list3[27]
            if abs(X)==0:
                planetgearsdr(m,z,ap,aaok,anchoeng, newComp)
                fichatecnica(aaok,False,False,False,False,False,m,ap,z,0,0,0,0,0, newComp)
                #numerop = spurgi
                htl(8)
            else:
                prof=skeng2(x,y,x2,y2,rva,rvf,aok,rb,m,z,ap,X, newComp)
                op='Cut'
                extruir(prof[0],anchoeng, newComp, 'NewBody')
                diente=extruir(prof[1],anchoeng,newComp, op)
                cpattern(1,False,ra,rf,z,diente,True,anchoeng, newComp, 'Cut')
                sk=newComp.sketches.add(newComp.xYConstructionPlane)
                origen=adsk.core.Point3D.create(0,0,0)
                sk.sketchCurves.sketchCircles.addByCenterRadius(origen,(m*z+2*X*m-2.5*m)/20)
                porf=sk.profiles.item(0)
                extruir(porf,anchoeng, newComp, 'Join')
                #numerop=7
                fichatecnica(aaok,False,False,False,False,True,m,ap,z,0,0,0,0,X, newComp)
                htl(9)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
        showhiddenbodies(hb, newComp)
#Spur PS

class cmdDef10OKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        eventArgs=adsk.core.CommandEventArgs.cast(args)
        app=adsk.core.Application.get()
        ui=app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.activeComponent
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        newComp = occ.component
        inputs2=eventArgs.command.commandInputs

        aaok=inputs2.itemById('aok10').value
        vul=inputs2.itemById('BoolValue10').value
        vul2=inputs2.itemById('BoolValue102').value
        mult1=1
        if vul2==True:
            mult1=2
        z=inputs2.itemById('IntegerSpinner10').value
        # q=inputs2.itemById('DropDownCommandInput10').selectedItem.name
        anchoeng=inputs2.itemById('ValueInput10').value/mult1
        m=inputs2.itemById('Module').value*10
        # a=str(q[0:len(q)-3])
        # m=float(a)
        ap=inputs2.itemById('FloatSpinner10').value
        ah=inputs2.itemById('FloatSpinner102').value
        X=inputs2.itemById('FloatSpinner103').value

        hb=hidebodies(newComp)
        list3=parameters(m, z, ap, ah,1.25*anchoeng,vul,X,aaok)
        rf=list3[0]
        x=list3[1]
        y=list3[2]
        x2=list3[3]
        y2=list3[4]
        aok=list3[5]
        Ttda=list3[6]
        ra=list3[7]
        zl=list3[13]
        xl=list3[20]
        yl=list3[21]
        aph=list3[26]
        rva=list3[28]
        rvf=list3[29]
        rb=list3[27]
        if abs(X)==0:
            helicalgs(aaok, vul, vul2, z, anchoeng, m, ap, ah, newComp)
            if vul2:
                fichatecnica(aaok,False,True,False,False,False,m,ap,z,ah,0,0,0,0, newComp)
                htl(11)
            else:
                fichatecnica(aaok, False, True, False, False, False, m, ap, z, ah, 0, 0, 0, 0, newComp)
                htl(8)
        else:
            prof=skeng2(x,y,x2,y2,rva,rvf,aok,rb,m,z,ap,X, newComp)
            op='Cut'
            esPS=True
            extruir(prof[0],anchoeng, newComp, 'NewBody')
            try:
                diente=helixext(esPS,rva,rvf,False,ah,prof[2],aok,xl,yl,zl,1.25*anchoeng,vul, newComp, op)
                cpattern(1,False,ra,rf,z,diente[0],True,anchoeng, newComp, 'Cut')
                if vul2==True:
                    mirror(prof[2],anchoeng,False, newComp)
                    sk = newComp.sketches.add(newComp.xYConstructionPlane)
                    origen = adsk.core.Point3D.create(0, 0, 0)
                    sk.sketchCurves.sketchCircles.addByCenterRadius(origen, (m * z + 2 * X * m - 2.5 * m) / 20)
                    porf = sk.profiles.item(0)
                    extruir(porf,2*anchoeng, newComp, 'Join')
                    fichatecnica(aaok, False, True, False, False, True, m, ap, z, ah, 0, 0, 0, X, newComp)
                    htl(12)
                else:
                    sk = newComp.sketches.add(newComp.xYConstructionPlane)
                    origen = adsk.core.Point3D.create(0, 0, 0)
                    sk.sketchCurves.sketchCircles.addByCenterRadius(origen, (m * z + 2 * X * m - 2.5 * m) / 20)
                    porf = sk.profiles.item(0)
                    extruir(porf, anchoeng, newComp, 'Join')
                    fichatecnica(aaok, False, True, False, False, True, m, ap, z, ah, 0, 0, 0, X, newComp)
                    htl(9)
            except:
                if ui:
                    ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
        showhiddenbodies(hb, newComp)
# helical PS

class cmdDef11OKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        eventArgs=adsk.core.CommandEventArgs.cast(args)
        app=adsk.core.Application.get()
        ui=app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.activeComponent
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        newComp = occ.component
        #aqui me quede
        inputs2=eventArgs.command.commandInputs
        try:
            aaok=inputs2.itemById('aok11').value
            vul=inputs2.itemById('BoolValue11').value
            vul2=False
            mult1=1
            if vul2==True:
                mult1=2
            z=inputs2.itemById('IntegerSpinner11').value
            largotornillo=inputs2.itemById('IntegerSpinner112').value*10
            # q=inputs2.itemById('DropDownCommandInput11').selectedItem.name
            anchoeng=inputs2.itemById('ValueInput11').value/mult1
            m=inputs2.itemById('Module').value*10
            # a=str(q[0:len(q) - 3])
            # m=float(a)
            ap=inputs2.itemById('FloatSpinner11').value
            radio=inputs2.itemById('FloatSpinner112').value
            tipo=inputs2.itemById('Brow11').selectedItem.name

            if tipo=='Helical':
                hb = hidebodies(newComp)
                worm=wormhelix(m,z,ap,radio*10,vul,largotornillo,vul, newComp)
                linearpattern(worm[1],worm[0],worm[6],worm[0], newComp)
                linearpattern(worm[5], worm[0],worm[6], worm[0], newComp)
                combine(newComp.bRepBodies.count-1, newComp)
                extruir(worm[7],(mt.pi*m/1), newComp,'Cut')
                extruir(worm[8],(mt.pi*m/1), newComp, 'Cut')
                extruir(worm[7],(.001), newComp, 'Join')
                showhiddenbodies(hb, newComp)
                hb2 = hidebodies(newComp)
                moveLastComponentBody(-worm[4]/20,((10*radio+4.5*m+(m*z/2+m))/10),(10*radio)/10, newComp)
                heliwormg(m,z,ap,worm,anchoeng,vul,vul2,aaok, newComp)
                showhiddenbodies(hb2, newComp)
                fichatecnica(aaok,False,True,False,True,False,m,ap,z,worm[3],0,0,radio,0, newComp)
                htl(24)

            elif tipo=='Hobbed Straight':
                hb = hidebodies(newComp)

                worm=wormhelix(m, z, ap, radio * 10, vul, largotornillo, vul, newComp)
                linearpattern(worm[1], worm[0], worm[6], worm[0], newComp)
                linearpattern(worm[5], worm[0],worm[6], worm[0], newComp)
                combine(newComp.bRepBodies.count-1, newComp)
                extruir(worm[7], (mt.pi * m / 1), newComp, 'Cut')
                extruir(worm[8], (mt.pi * m / 1), newComp, 'Cut')
                extruir(worm[7], (.001), newComp, 'Join')
                moveLastComponentBody(-worm[4] / 20, ((10 * radio + 4.5 * m + (m * z / 2 + m)) / 10), (10 * radio) / 10, newComp)
                showhiddenbodies(hb, newComp)

                hb2=hidebodies(newComp)
                listan = wormhelix2(m, z, ap, radio * 10, aaok, largotornillo, vul,anchoeng, newComp)
                feature =listan[0]
                feature2=listan[4]
                linecenter=listan[1]
                posx=listan[2]
                posy=listan[3]
                indcpattern(feature,linecenter,z, newComp)
                indcpattern(feature2, linecenter, z, newComp)
                moveLastComponentBody(-posx,posy/10,anchoeng/2, newComp)
                showhiddenbodies(hb2, newComp)
                fichatecnica(aaok, False, True, False, True, False, m, ap, z, worm[3], 0, 0, radio, 0, newComp)
                htl(29)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
# Worm gear

tbPanel = None
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        workSpace = ui.workspaces.itemById('FusionSolidEnvironment')
        tbPanels = workSpace.toolbarPanels

        global tbPanel
        tbPanel = tbPanels.itemById('NewPanel')
        if tbPanel:
            tbPanel.deleteMe()
        tbPanel = tbPanels.add('NewPanel', 'GF GEAR GENERATOR', 'SelectPanel', False)
        #En la línea anterior añade un tbPanel :toolbarTabPanels_var.add(id, name, positionID, isBefore)
        #id y name ya las conozco, positionId es el panel de relacion e isBefore te dice si va antes o despues del panel de positionId

        #TBPANEL 1
        # Empty panel can't be displayed. Add a command to the panel
        cmdDef =ui.commandDefinitions.itemById('NC1')
        cmdDef2=ui.commandDefinitions.itemById('NC2')
        cmdDef3=ui.commandDefinitions.itemById('NC3')
        cmdDef4=ui.commandDefinitions.itemById('NC4')
        cmdDef5=ui.commandDefinitions.itemById('NC5')
        cmdDef6=ui.commandDefinitions.itemById('NC6')
        cmdDef7=ui.commandDefinitions.itemById('NC7')
        cmdDef8=ui.commandDefinitions.itemById('NC8')
        cmdDef9=ui.commandDefinitions.itemById('NC9')
        cmdDef10=ui.commandDefinitions.itemById('NC10')
        cmdDef11=ui.commandDefinitions.itemById('NC11')
        if cmdDef:
            cmdDef.deleteMe()
        if cmdDef2:
            cmdDef2.deleteMe()
        if cmdDef3:
            cmdDef3.deleteMe()
        if cmdDef4:
            cmdDef4.deleteMe()
        if cmdDef5:
            cmdDef5.deleteMe()
        if cmdDef6:
            cmdDef6.deleteMe()
        if cmdDef7:
            cmdDef7.deleteMe()
        if cmdDef8:
            cmdDef8.deleteMe()
        if cmdDef9:
            cmdDef9.deleteMe()
        if cmdDef10:
            cmdDef10.deleteMe()
        if cmdDef11:
            cmdDef11.deleteMe()
        cmdDef=ui.commandDefinitions.addButtonDefinition('NC1', 'Spur Gear', 'Creates a standard spur gear.','Resources/Recto')
        cmdDef2=ui.commandDefinitions.addButtonDefinition('NC2','Internal Spur Gear','Creates a standard internal spurg gear.','Resources/InteriorGear')
        cmdDef3=ui.commandDefinitions.addButtonDefinition('NC3','Non-Standard Internal Spur Gear','Creates a non-standard internal spur gear.','Resources/NoStdr')
        cmdDef4=ui.commandDefinitions.addButtonDefinition('NC4','Simple/Double Helical Gear','Creates a standard simple/double helical gear. ','Resources/Helical')
        cmdDef5=ui.commandDefinitions.addButtonDefinition('NC5','Simple/Double Non-Standard Internal Helical Gear.','Creates a non-standard simple/double internal helical gear.','Resources/NoStdr')
        cmdDef6=ui.commandDefinitions.addButtonDefinition('NC6','Simple/Double Internal Helical Gear.','Creates a standard simple/double internal helical gear.','Resources/InteriorGear')
        cmdDef7=ui.commandDefinitions.addButtonDefinition('NC7','Helical/Straight Gear Rack','Creates a gear rack.','Resources/Rack')
        cmdDef8=ui.commandDefinitions.addButtonDefinition('NC8','90° Bevel Gears','Creates a standard pair of bevel gears with intersecting axes at 90°.','Resources/Conicos')
        cmdDef9=ui.commandDefinitions.addButtonDefinition('NC9','Profile Shifted Spur Gear','Creates a spur gear using "Profile Shifting".','Resources/Recto')
        cmdDef10=ui.commandDefinitions.addButtonDefinition('NC10','Profile Shifted Helical Gear','Creates a helical gear using "Profile Shifting".','Resources/Helical')
        cmdDef11=ui.commandDefinitions.addButtonDefinition('NC11','Worm Gear Drive','Creates a worm gear drive with its screw.','Resources/WormGear')
        #Sin lo siguiente ni los botones ni el panel aparecen
        cmdDefcontrol=tbPanel.controls.addCommand(cmdDef)
        tbPanel.controls.addCommand(cmdDef4)
        tbPanel.controls.addCommand(cmdDef8)
        tbPanel.controls.addCommand(cmdDef7)
        tbPanel.controls.addCommand(cmdDef11)
        tbPanel.controls.addSeparator()
        tbPanel.controls.addCommand(cmdDef2)
        tbPanel.controls.addCommand(cmdDef3)
        tbPanel.controls.addCommand(cmdDef6)
        tbPanel.controls.addCommand(cmdDef5)
        tbPanel.controls.addSeparator()
        tbPanel.controls.addCommand(cmdDef9)
        tbPanel.controls.addCommand(cmdDef10)
        cmdDefcontrol.isPromotedByDefault=True
        cmdDefcontrol.isPromoted=True
        #vinculo los botones a su comando
        cmdDefPressed=cmdDefPressedEventHandler()
        cmdDef.commandCreated.add(cmdDefPressed)
        handlers.append(cmdDefPressed)

        cmdDef2Pressed=cmdDef2PressedEventHandler()
        cmdDef2.commandCreated.add(cmdDef2Pressed)
        handlers.append(cmdDef2Pressed)

        cmdDef3Pressed=cmdDef3PressedEventHandler()
        cmdDef3.commandCreated.add(cmdDef3Pressed)
        handlers.append(cmdDef3Pressed)

        cmdDef4Pressed=cmdDef4PressedEventHandler()
        cmdDef4.commandCreated.add(cmdDef4Pressed)
        handlers.append(cmdDef4Pressed)

        cmdDef5Pressed=cmdDef5PressedEventHandler()
        cmdDef5.commandCreated.add(cmdDef5Pressed)
        handlers.append(cmdDef5Pressed)

        cmdDef6Pressed=cmdDef6PressedEventHandler()
        cmdDef6.commandCreated.add(cmdDef6Pressed)
        handlers.append(cmdDef6Pressed)

        cmdDef7Pressed=cmdDef7PressedEventHandler()
        cmdDef7.commandCreated.add(cmdDef7Pressed)
        handlers.append(cmdDef7Pressed)

        cmdDef8Pressed=cmdDef8PressedEventHandler()
        cmdDef8.commandCreated.add(cmdDef8Pressed)
        handlers.append(cmdDef8Pressed)

        cmdDef9Pressed=cmdDef9PressedEventHandler()
        cmdDef9.commandCreated.add(cmdDef9Pressed)
        handlers.append(cmdDef9Pressed)

        cmdDef10Pressed=cmdDef10PressedEventHandler()
        cmdDef10.commandCreated.add(cmdDef10Pressed)
        handlers.append(cmdDef10Pressed)

        cmdDef11Pressed=cmdDef11PressedEventHandler()
        cmdDef11.commandCreated.add(cmdDef11Pressed)
        handlers.append(cmdDef11Pressed)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
def stop(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        if tbPanel:
            tbPanel.deleteMe()
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

