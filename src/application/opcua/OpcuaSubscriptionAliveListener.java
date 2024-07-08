package application.opcua;

import com.prosysopc.ua.client.Subscription;
import com.prosysopc.ua.client.SubscriptionAliveListener;

public class OpcuaSubscriptionAliveListener implements SubscriptionAliveListener{

	@Override
	public void onAfterCreate(Subscription s) {
		
	}

	@Override
	public void onAlive(Subscription s) {
		
	}

	@Override
	public void onLifetimeTimeout(Subscription s) {
		
	}

	@Override
	public void onTimeout(Subscription s) {
		
	}

}
