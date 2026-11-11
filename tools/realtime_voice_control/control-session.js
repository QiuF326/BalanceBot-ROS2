/* Reconnect the session and role only. Never replay motion, reset or microphone. */
(function(root,factory){
  if(typeof module==='object'&&module.exports)module.exports=factory();
  else root.ControlSession=factory();
})(typeof globalThis!=='undefined'?globalThis:this,function(){
  'use strict';
  const roles=['FULL','LINEAR','ANGULAR','LEFT','RIGHT'];
  const key='balancecar.connection.v1';
  return class ControlSession{
    constructor({api,storage=null,beforeConnect=async()=>{},now=()=>Date.now()}){
      this.api=api;this.storage=storage;this.beforeConnect=beforeConnect;this.now=now;
      this.connected=false;this.busy=null;this.generation=0;this.retryAt=0;
      let saved={};try{saved=JSON.parse(storage&&storage.getItem(key))||{};}catch(e){}
      this.role=roles.includes(saved.role)?saved.role:null;
      this.enabled=saved.resume===true;
    }
    save(){try{if(this.storage)this.storage.setItem(key,JSON.stringify({role:this.role,resume:this.enabled}));}catch(e){}}
    observe(state){this.connected=!!state.connected;}
    lost(){this.connected=false;this.retryAt=this.now()+1000;}
    shouldReconnect(){return this.enabled&&!this.connected&&!this.busy&&this.now()>=this.retryAt;}
    connect(role=this.role){
      if(role!==null&&!roles.includes(role))return Promise.reject(new Error('控制角色无效'));
      if(this.busy)return this.busy;
      this.enabled=true;this.role=role;this.save();
      const generation=++this.generation;
      const active=()=>generation===this.generation&&this.enabled;
      const run=async()=>{
        await this.beforeConnect(this.connected);
        if(!active())return;
        const state=await this.api('connect');
        if(!active())return;
        // A connection that survived a brief interruption may retain old input.
        // Release it before restoring the preferred role, including the same role.
        if(state.my_slot)await this.api('release-role');
        if(!active())return;
        if(role)await this.api('claim',{role});
        this.retryAt=0;
      };
      this.busy=run().catch(error=>{this.retryAt=this.now()+1500;throw error;})
        .finally(()=>{this.busy=null;});
      return this.busy;
    }
    async disconnect(){
      this.enabled=false;this.save();++this.generation;
      if(this.busy)await this.busy.catch(()=>{});
      return this.api('leave');
    }
    async release(){
      this.role=null;this.save();++this.generation;
      if(this.busy)await this.busy.catch(()=>{});
      return this.api('release-role');
    }
  };
});
