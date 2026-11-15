<?php
// A robot on the core binding supplied by the arena.
// Tracking, geometry, protocol parsing and serialization live in the core.
// Bearing is earned: a scan contact's bearing is off by up to its
// bearingError (degrees, a fixed share of the arc), so a wide beam finds a
// robot and a narrow one aims at it. This robot narrows the beam on its target
// and shoots along a line fitted through its recent sightings. Contacts within
// 120 units are exact.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
//   from_mount(heading pitch) mount_toward(point)
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   history(slot) last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) to_chassis(heading pitch attitude)
//   from_chassis(heading pitch attitude) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
require 'clunkyard.php';
use Clunkyard as cy;

final class Starter extends cy\Robot {
    private const SEARCH_ARC=40;  // degrees: sweeps the circle in half a second
    private const HOLD_WIDTH=60;  // units across at the target: it stays in the beam from tick to tick
    private const MIN_ARC=6;      // degrees: the narrowest the beam holds
    private const WINDOW=12;      // ticks of sightings the aim averages
    private const SIGHTINGS=8;    // narrow sightings in the window before a shot
    private const LOOK_EVERY=40;  // ticks between looks around the target, for a nearer robot
    private const REACQUIRE=10;   // ticks to look again where a lost target was
    private bool $free=false;private int $dodges=0;private int $dodgeEnd=0;private int $slot=0;
    private function steer(float $towards,float $current):float{return $this->angles->clamp($this->angles->angleDiff($towards,$current)/45);}
    public function onStart():void{$this->free=$this->match->is3d()&&$this->arena->gravity()->z==0;}
    // A straight line through the narrowest of the last WINDOW ticks' sightings:
    // where the robot is now, how fast it moves, and how many sightings it rests
    // on. Each sighting is off by its bearingError; the line averages that out.
    private function fit(int $slot,int $tick):?array{
        $recent=array_values(array_filter($this->tracker->history($slot),fn($s)=>$s->tick>$tick-self::WINDOW));
        if(count($recent)<2)return null;
        $best=min(array_map(fn($s)=>$s->bearingError,$recent));
        $seen=array_values(array_filter($recent,fn($s)=>$s->bearingError<=1.5*$best));$n=count($seen);
        if($n<2)return null;
        $t0=array_sum(array_map(fn($s)=>$s->tick,$seen))/$n;
        $spread=array_sum(array_map(fn($s)=>($s->tick-$t0)**2,$seen));
        $position=[];$velocity=[];
        foreach(['x','y','z'] as $axis){
            $mean=array_sum(array_map(fn($s)=>$s->position->$axis,$seen))/$n;
            $slope=array_sum(array_map(fn($s)=>($s->tick-$t0)*($s->position->$axis-$mean),$seen))/$spread;
            $position[]=$mean+$slope*($tick-$t0);$velocity[]=$slope*$this->match->tickRate();
        }
        return [new cy\Vec3(...$position),new cy\Vec3(...$velocity),$n];
    }
    public function onTurn():void{
        $me=$this->me;$position=$me->position();$gun=$this->gun;$scanner=$this->scanner;$angles=$this->angles;$tick=$this->clock->tick();
        $this->drive->forces(brake:0.2);
        $target=$this->sensors->nearest();$cue=$this->tracker->cue();
        $lastSeen=$this->slot?$this->tracker->lastSeen($this->slot):null;
        if($target){
            $this->slot=$target->slot;
            $throttle=$target->range>350?0.7:($target->range<200?-0.4:0.2);
            $this->drive->set(throttle:$throttle,turn:$this->steer($target->bearing,$me->heading()),pitchTurn:$this->steer($target->elevation,$me->pitch()));
            // Halve the beam each tick until it is HOLD_WIDTH across at the target:
            // narrow enough for a precise bearing, wide enough to keep it in.
            $hold=max(self::MIN_ARC,rad2deg(2*atan2(self::HOLD_WIDTH/2,$target->range)));
            // In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks,
            // widen it for a tick: a nearer robot it finds becomes the target.
            $arc=$tick%self::LOOK_EVERY===0?self::SEARCH_ARC:max($hold,min(self::SEARCH_ARC,$scanner->last()->arc/2));
            $scanner->set($target->bearing,$arc,$target->elevation);
            $power=$gun->powerForFlightTime($target->range,1);
            // An exact contact: gun.lead is the intercept if the target holds
            // course. A noisy one: lead the fitted line, and fire only once it
            // rests on enough sightings.
            $ready=true;
            if($target->bearingError>0){
                $fit=$this->fit($target->slot,$tick);
                $aim=$fit?$gun->leadPoint($fit[0],$fit[1],$power):null;
                $ready=$fit!==null&&$fit[2]>=self::SIGHTINGS;
            }else $aim=$gun->lead($target,$power);
            $heading=$aim?->heading??$target->bearing;$pitch=$aim?->pitch??$target->elevation;
            $gun->aim($heading,$pitch);$now=$gun->aimNow();
            if($ready&&abs($angles->angleDiff($heading,$now->heading))<3&&abs($pitch-$now->pitch)<3&&$gun->canFire($power,10))$gun->fire($power);
            $this->status->set($ready?'TRACK':'NARROW',$target->range);
        }elseif($lastSeen!==null&&$tick-$lastSeen<=self::REACQUIRE){
            // Lost it: look again where it was last seen, widening the beam, before searching.
            $where=$this->tracker->position($this->slot);
            $bearing=$angles->bearingTo($position,$where);$elevation=$this->free?$angles->elevationTo($position,$where):0;
            $this->drive->set(0.5,$this->steer($bearing,$me->heading()),$this->steer($elevation,$me->pitch()));
            $scanner->set($bearing,min(self::SEARCH_ARC,$scanner->last()->arc*2),$elevation);
            $this->status->set('REACQUIRE');
        }elseif($cue){
            $bearing=$angles->bearingTo($position,$cue->position);$distance=$angles->distance($position,$cue->position);
            $elevation=$this->free?$angles->elevationTo($position,$cue->position):0;$error=min($cue->bearingError,30);
            $across=($tick%5-2)/2*$error;$up=$this->free?(intdiv($tick,5)%3-1)*$error/2:0;
            $this->drive->set(0.7,$this->steer($bearing,$me->heading()),$this->steer($elevation,$me->pitch()));
            $scanner->set($bearing+$across,$scanner->arcForReach($distance+100),$elevation+$up);$this->status->set('CUE',$distance);
        }else{
            $this->drive->set(0.5,0.33,$this->steer(0,$me->pitch()));
            $pitch=$this->free?(intdiv($tick,10)%3-1)*35:0;
            $scanner->set($angles->normalize($scanner->last()->heading+36),self::SEARCH_ARC,$pitch);$this->status->set('SEARCH');
        }
        // A simple dodge: vary it, or predict the shooter.
        if($tick>=$this->dodgeEnd&&array_filter($this->sensors->cues(),fn($c)=>$c->source==='muzzle')){$this->dodges++;$this->dodgeEnd=$tick+10;}
        if($tick<$this->dodgeEnd){
            // Off the line of fire: left, then right next time (turn is positive left, strafe right).
            $left=$this->dodges%2===1;
            if($this->match->control()==='forces'){$this->drive->set(throttle:0.5);$this->drive->forces(strafe:$left?-1:1);}
            else $this->drive->set(throttle:1,turn:$left?1:-1);
            $this->status->set($left?'dodge:left':'dodge:right',$this->dodges);
        }
        if($this->battery->charge()<$this->battery->capacity()/4)$this->drive->set(throttle:$this->drive->get('throttle')*0.5);
        $storm=$this->nav->storm();$zone=$this->nav->zone();
        if($storm&&($storm->margin($position)??-1e9)>-60){
            $away=$angles->bearingTo($storm->center,$position);
            $this->drive->set(throttle:0.8,turn:$angles->clamp($angles->angleDiff($away,$me->heading())/30));
        }
        if($zone&&($zone->margin($position)??1e9)<60){
            $climb=$zone->shape==='sphere'?$angles->elevationTo($position,$zone->center):0;
            $this->drive->set(0.8,$angles->clamp($angles->angleDiff($angles->bearingTo($position,$zone->center),$me->heading())/30),$angles->clamp(($climb-$me->pitch())/30));
        }
    }
}


cy\run(new Starter());
