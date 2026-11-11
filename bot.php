<?php
// A robot on the core binding supplied by the arena.
// Tracking, geometry, protocol parsing and serialization live in the core.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range)
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
require 'clunkyard.php';
use Clunkyard as cy;

final class Starter extends cy\Robot {
    private bool $free=false;
    private function steer(float $towards,float $current):float{return $this->angles->clamp($this->angles->angleDiff($towards,$current)/45);}
    public function onStart():void{$this->free=$this->match->is3d()&&$this->arena->gravity()->z==0;}
    public function onTurn():void{
        $me=$this->me;$position=$me->position();$gun=$this->gun;$scanner=$this->scanner;$angles=$this->angles;
        $this->drive->forces(brake:0.2);
        $target=$this->sensors->nearest();$cue=$this->tracker->cue();
        if($target){
            $throttle=$target->range>350?0.7:($target->range<200?-0.4:0.2);
            $this->drive->set(throttle:$throttle,turn:$this->steer($target->bearing,$me->heading()),pitchTurn:$this->steer($target->elevation,$me->pitch()));
            $scanner->set($target->bearing,20,$target->elevation);
            $power=$gun->powerForFlightTime($target->range,1);$aim=$gun->lead($target,$power);
            $heading=$aim?->heading??$target->bearing;$pitch=$aim?->pitch??$target->elevation;
            $gun->aim($heading,$pitch);$now=$gun->aimNow();
            if(abs($angles->angleDiff($heading,$now->heading))<3&&abs($pitch-$now->pitch)<3&&$gun->canFire($power,10))$gun->fire($power);
            $this->status->set('TRACK',$target->range);
        }elseif($cue){
            $bearing=$angles->bearingTo($position,$cue->position);$distance=$angles->distance($position,$cue->position);
            $elevation=$this->free?$angles->elevationTo($position,$cue->position):0;$error=min($cue->bearingError,30);$tick=$this->clock->tick();
            $across=($tick%5-2)/2*$error;$up=$this->free?(intdiv($tick,5)%3-1)*$error/2:0;
            $this->drive->set(0.7,$this->steer($bearing,$me->heading()),$this->steer($elevation,$me->pitch()));
            $scanner->set($bearing+$across,$scanner->arcForReach($distance+100),$elevation+$up);$this->status->set('CUE',$distance);
        }else{
            $this->drive->set(0.5,0.33,$this->steer(0,$me->pitch()));
            $pitch=$this->free?(intdiv($this->clock->tick(),10)%3-1)*35:0;
            $scanner->set($angles->normalize($scanner->last()->heading+36),40,$pitch);$this->status->set('SEARCH');
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
        // The arena's rate calibration only; safe to delete.
        $work=$this->raw->number('state.calibration.work');
        if($work!==null&&$work>0)$this->status->set('WORK',calibrationWork((int)$work,$this->clock->tick()));
    }
}

// The calibration kernel, xorshift-sqrt v1: work rounds of a 64-bit xorshift
// and a square-root sum. PHP's integers are signed and its >> is arithmetic,
// so the masks make the right shifts logical.
function calibrationWork(int $work, int $tick): int
{
    $x = 88172645463325252 ^ $tick;
    $sum = 0.0;
    for ($i = 0; $i < $work; $i++) {
        $x ^= $x << 13;
        $x ^= ($x >> 7) & 0x01FFFFFFFFFFFFFF;
        $x ^= $x << 17;
        $sum += sqrt(($x >> 11) & 0x001FFFFFFFFFFFFF);
    }
    return ((int) floor($sum) + (($x >> 40) & 0xFFFFFF)) % 1000000;
}

cy\run(new Starter());
