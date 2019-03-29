var initId = 0;
var world;
var ctx;
var canvasWidth;
var canvasHeight;
var canvasTop;
var canvasLeft;


var effectsList =  new Array();

var gameMaster = function(){
    var robotList = new Array();
    var teamShare = new Array(2);
    var teamScore = new Array(2);
    teamScore[0]=0;
    teamScore[1]=0;
    return {
        registerRobot: function(robot){
            robotList.push(robot);
        },
        getRobotList: function(){
            return robotList;
        },
        destroyRobot: function(robot){
            world.DestroyJoint(robot.motor);
            world.DestroyBody(robot.body);
            world.DestroyBody(robot.wheel);
            effectsList.push({
                type:'explode',
                pos: (robot.wheel.m_position.Copy()),
                duration:30
            });
            robotList=robotList.without(robot);
        },
        Step: function(timeStep){
            var deathNote = new Array();
            var teamShareNow = new Array();
            teamShareNow[0]=0;
            teamShareNow[1]=0;
            robotList.each(function(robot,idx){
                var u;
                var info = {
                    'timeStep':timeStep
                };
                var position = robot.wheel.m_position.Copy();
                var velocity = robot.wheel.m_linearVelocity.Copy();
                var bodyAngularVelocity = robot.body.m_angularVelocity;
                var motorVelocity = robot.wheel.m_angularVelocity-bodyAngularVelocity;
                var bodyAttitude =  robot.body.m_rotation;
                try{
                    u = robot.controller(position,velocity,bodyAttitude,bodyAngularVelocity,motorVelocity,info);
                }catch(e){
                    u=0;
                }
                robot.motor.SetMotorTorque(Math.abs(u));
                robot.motor.SetMotorSpeed(100000*u/Math.abs(u));
                var contactlist=robot.body.GetContactList();
                if(position.x<-20 || position.x>canvasWidth+20 || position.y>canvasHeight+50){
                    deathNote.push(robot);
                }else{
                    while(contactlist!==null){
                        if(contactlist.contact.GetShape1().m_body.destructive===true || 
                           contactlist.contact.GetShape2().m_body.destructive===true){
                            deathNote.push(robot);
                            break;
                        }
                        contactlist=contactlist.next;
                    }
                }
                teamShareNow[robot.team-1]++;
                var contactlist=robot.wheel.GetContactList();
                while(contactlist!==null){
                    if(contactlist.contact.GetShape1().m_body.destructive===true){
                        var score=contactlist.contact.GetShape1().m_body.m_position.y;
                        if(typeof(robot.score)=='undefined' || robot.score<score){
                            teamScore[robot.team-1]+=score;
                            effectsList.push({
                                'type':'score',
                                'pos': (robot.body.m_position.Copy()),
                                duration:30,
                                'score': score,
                                'team': robot.team
                            });
                            robot.score=score;
                            break;													
                        }
                    }
                    if(contactlist.contact.GetShape2().m_body.destructive===true){
                        var score=contactlist.contact.GetShape2().m_body.m_position.y;
                        if(typeof(robot.score)=='undefined' || robot.score<score){
                            teamScore[robot.team-1]+=score;
                            effectsList.push({
                                'type':'score',
                                'pos': (robot.body.m_position.Copy()),
                                duration:30,
                                'score': score,
                                'team': robot.team
                            });
                            robot.score=score;
                            break;													
                        }
                    }
                    if(contactlist.contact.GetShape1().m_body.goal===true||contactlist.contact.GetShape2().m_body.goal===true){
                        if(typeof(robot.goalcount)=='undefined'){
                            robot.goalcount=150;
                        }
                        robot.goalcount--;
                        if(robot.goalcount<0){
                            robot.goalcount=150;
                            var score=100;
                            teamScore[robot.team-1]+=score;
                            effectsList.push({
                                'type':'score',
                                'pos': (robot.body.m_position.Copy()),
                                duration:30,
                                'score': score,
                                'team': robot.team
                            });
                        }
                    }	
                    contactlist=contactlist.next;
                }
                
            });
            var gmins=this;
            deathNote.each(function(robot){
                gmins.destroyRobot(robot);
            });
            teamShare=teamShareNow;
        },
        getTeamShare: function(){
            return teamShare;
        },
        getTeamScore: function(){
            return teamScore;
        }
        
    };
}

var gm = new gameMaster();


function createWorld() {
    var worldAABB = new b2AABB();
    worldAABB.minVertex.Set(-canvasWidth*1.5, -canvasHeight*1.5);
    worldAABB.maxVertex.Set(canvasWidth*1.5, canvasHeight*1.5);
    var gravity = new b2Vec2(0, 600);
    var doSleep = true;
    var w = new b2World(worldAABB, gravity, doSleep);
    
    var sd = new b2BoxDef();
    sd.extents.Set(440/960*canvasWidth, 10);
    sd.density = 1.0;
    
    var bd = new b2BodyDef();
    bd.AddShape(sd);
    bd.position.Set(canvasWidth/2, Math.round(canvasHeight*(1-80/1280)));
    var body = w.CreateBody(bd);
    body.destructive=true;
    body.goal=true;
    body.strokeStyle='#00aa00';
    body.fillStyle='#aaeeaa';
    
    var rjd = new b2RevoluteJointDef();
    
    rjd.anchorPoint.Set(canvasWidth/2, Math.round(canvasHeight*(1-80/1280)));
    rjd.body1 = w.m_groundBody;
    rjd.body2 = body;
    var rjd=w.CreateJoint(rjd);
    createBox(w, 800/960*canvasWidth, canvasHeight*(1-10/1280), 15/960*canvasWidth, 15,true,0,true);
    createBox(w, 200/960*canvasWidth, canvasHeight*(1-10/1280), 15/960*canvasWidth, 15,true,0,true);
    
    var xold3=0.5,xold2=0.5,xold1=0.5;
    var dy=100;
    for(var y=150;y<canvasHeight-250;y+=dy){
        var xr=Math.random();
        if(Math.abs(xold1-xr)<0.2||Math.abs(xold2-xr)<0.2||Math.abs(xold3-xr)<0.1){
            y-=dy;
            continue;
        }
        xold3=xold2;
        xold2=xold1;
        xold1=xr;
        var ang=-(xr-0.5)*(Math.random()-0.2)*(1+0.2*(canvasWidth-960)/960);
        createBox(w,(xr)*canvasWidth, y, 40*(1+(canvasWidth-960)/960*0.4)*(Math.random()+2), 5,true,ang,true);		
    }
    return w;
}


function createBall(world, x, y,rad,density,restitution) {
    var ballSd = new b2CircleDef();
    if (typeof(rad) == 'undefined') rad = 20;
    if (typeof(density) == 'undefined') density = 1.0;
    if (typeof(restitution) == 'undefined') restitution = 0.3;
    
    ballSd.density = density;
    ballSd.radius = rad;
    ballSd.restitution = restitution;
    ballSd.friction = 1.0;
    var ballBd = new b2BodyDef();
    ballBd.AddShape(ballSd);
    ballBd.position.Set(x,y);
    return world.CreateBody(ballBd);
}

function createRobot(world,gm, x,y, team){
    var ballSd = new b2CircleDef();
    ballSd.density = 0.4;
    ballSd.radius = 15;
    ballSd.restitution = 0.1;
    ballSd.friction = 10;
    var ballBd = new b2BodyDef();
    ballBd.AddShape(ballSd);
    ballBd.position.Set(x,y);
    var wheel = world.CreateBody(ballBd);
    var sd = new b2BoxDef();
    if(typeof(team)=='undefined') team=1;
    sd.extents.Set(5, 30);
    sd.density = 1.0;
    
    var sd = new b2BoxDef();
    sd.extents.Set(4, 30);
    sd.density = 1.0;
    var bd = new b2BodyDef();
    bd.AddShape(sd);
    var motor = new b2RevoluteJointDef();
    motor.enableLimit=false;
    
    bd.position.Set(x, y-30);
    var body = world.CreateBody(bd);
    
    motor.anchorPoint.Set(x, y);
    motor.body1 = wheel;
    motor.body2 = body;
    motor.enableMotor = true;
    var motor=world.CreateJoint(motor);
    
    var robot={
        'wheel':wheel,
        'body':body,
        'motor':motor,
        controller: (function(){
            var controller;
            try{
                if(team==1){
                    controller=(function(pos,vel,phi,dphi,dth,info){
                        var u; // torque
                        
                        /* state feedback */
                        u = -10000000*phi-100000*dphi
                        -1000*(pos.x-480)-1000*(vel.x);
                        
                        return(u);
                    });
                }else{
                    /* static variables */
                    var K1,K2,K3,K4;
                    K1=20000000*(Math.random()+0.1);
                    K2=2000000*(Math.random()+0.1);
                    K3=200*(Math.random()+0.1);
                    K4=2000*(Math.random()+0.1);
                    
                    /* control law */
                    controller =
                        (function(pos,vel,phi,dphi,dth,info){
                            var u;
                            /* state feedback */
                            u = -K1*phi-K2*dphi
                            -K3*(pos.x-480)-K4*(vel.x);
                            
                            /* random action */
                            if(Math.random()<0.01){
                                u=10000000*(Math.random()-0.5);
                            }
                            return(u);
                        });
                }
            }catch(e){
                controller=(function(pos,vel,phi,dphi,dth,info){return(0);});
            }
            return controller;
        })()
    };
    robot.team=team;
    robot.wheel.probot=robot;
    robot.body.probot=robot;
    robot.motor.probot=robot;
    gm.registerRobot(robot);
}

function createBox(world, x, y, width, height, fixed,rot,destructive) {
    if (typeof(fixed) == 'undefined') fixed = true;
    if (typeof(rot) == 'undefined') rot=0;
    if (typeof(destructive) == 'undefined') destructive=false;
    var boxSd = new b2BoxDef();
    if (!fixed) boxSd.density = 1.0;
    boxSd.extents.Set(width, height);
    var boxBd = new b2BodyDef();
    boxBd.AddShape(boxSd);
    boxBd.position.Set(x,y);
    boxBd.rotation=rot;
    boxBd=world.CreateBody(boxBd)
    boxBd.destructive=destructive;
    return boxBd;
}


function doEffects(ctx){
    var finished=new Array();
    effectsList.each(function(ef){
        ef.duration--;
        if(ef.duration<=0){
            finished.push(ef);
        }
        ctx.beginPath();
        if(ef.type==='explode'){
            var pos=ef.pos;
            var r= (30-ef.duration)*10/3;
            ctx.strokeStyle = '#fff';
            ctx.fillStyle = 'rgba(200,200,0,'+ef.duration/30/2+')';
            ctx.arc(pos.x,pos.y,r,0,2*Math.PI,true);
            ctx.fill();
        }
        if(ef.type==='score'){
            var pos=ef.pos;
            if(ef.team===1){
                ctx.fillStyle ='rgba(0,0,255,'+ef.duration/30+')';
            }else{
                ctx.fillStyle ='rgba(255,0,0,'+ef.duration/30+')';
            }
            ctx.font = "900 20px AvenirNext,Verdana,sans-serif";
            ctx.textAlign = "center";
            ctx.fillText(ef.score,pos.x, pos.y-(30-ef.duration));
        }
    });
    finished.each(function(fe){
        effectsList=effectsList.without(fe);		
    });
}

var autogenRobotInterval=50,autogenRobotCounter=0;
var autogenStimulatorInterval=300,autogenStimulatorCounter=25;

function step(cnt) {
    var timeStep = 1.0/60;
    var iteration = 5;
    world.Step(timeStep, iteration);
    gm.Step();
    ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    drawWorld(world, ctx);
    doEffects(ctx);
    
    var score=gm.getTeamScore();
    ctx.font = '900 24px AvenirNext,Verdana,sans-serif';
    ctx.textAlign = "right";
    ctx.fillStyle ='rgba(0,0,0,0.6)';
    ctx.fillText("2D ROBOT SUMO",canvasWidth-10,canvasHeight-10);
    ctx.font = "900 24px AvenirNext,Verdana,sans-serif";
    
    ctx.textAlign = "right";
    ctx.fillStyle ='rgba(0,0,255,0.6)';
    ctx.fillText("Blue: "+score[0],canvasWidth-10,20);
    ctx.fillStyle ='rgba(255,0,0,0.6)';
    ctx.fillText("Red:  "+score[1],canvasWidth-10,40);
    
    if(autogenRobotCounter<0){
        autogenRobotCounter=autogenRobotInterval;
        autogenRobots();
    }
    autogenRobotCounter--;
    
    if(autogenStimulatorCounter<0){
        autogenStimulatorCounter=autogenStimulatorInterval;
        autogenStimulator();
    }
    autogenStimulatorCounter--;
    setTimeout('step()', 10);
}
function autogenRobots(){
    createRobot(world, gm,  Math.random()*canvasWidth,0,1);
    createRobot(world, gm,  Math.random()*canvasWidth,0,2);
}

var stimulatorList=new Array();


function autogenStimulator(){
    var bd=createBall(world,Math.random()*canvasWidth,0,12,40,0.6);
    stimulatorList.push(bd);
    if(stimulatorList.length>50){
        world.DestroyBody(stimulatorList.shift());
    }
}
window.onload = function(){  
    var canvasElm = $('canvas');
    
    var setDim=(function(){        
        jQuery("#canvas").attr({height:jQuery("#wrapper").height()*2});
        jQuery("#canvas").attr({width:jQuery("#wrapper").width()*2});
        jQuery("#canvas").height(jQuery("#wrapper").height());
        jQuery("#canvas").width(jQuery("#wrapper").width());
        
        canvasWidth = parseInt(canvasElm.width);
        canvasHeight = parseInt(canvasElm.height);
        canvasTop = parseInt(canvasElm.style.top);
        canvasLeft = parseInt(canvasElm.style.left);        
    });
    jQuery(window).resize(setDim);
    setDim();
    ctx = $('canvas').getContext('2d');
    
    world = createWorld();
    step();
    
};