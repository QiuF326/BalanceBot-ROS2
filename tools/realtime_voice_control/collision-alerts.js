/* Per-page alarm deduplication; both phones consume the same server event. */
(function(root,factory){
  if(typeof module==='object'&&module.exports)module.exports=factory();
  else root.CollisionNotifier=factory();
})(typeof globalThis!=='undefined'?globalThis:this,function(){
  'use strict';
  return class CollisionNotifier {
    constructor(){this.instance=null;this.seen=0;}
    observe(collision){
      if(!collision)return null;
      const event=collision.last_event;
      const id=event&&Number.isInteger(event.id)?event.id:0;
      if(this.instance!==collision.instance_id){
        this.instance=collision.instance_id;this.seen=id;return null;
      }
      if(id<=this.seen)return null;
      this.seen=id;
      return event;
    }
  };
});
