
#include <bt.h>
#include <behavior_tree.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BehaviorTree");
    try
    {
        int TickPeriod_milliseconds = 1000;

        // Level 1
        BT::SequenceNodeWithMemory* sequence1 = new BT::SequenceNodeWithMemory("seq 1");

        // Level 2
        BT::FallbackNodeWithMemory* fallback2A = new BT::FallbackNodeWithMemory("fall 2A");
        BT::FallbackNodeWithMemory* fallback2B = new BT::FallbackNodeWithMemory("fall 2B");

        // Level 3
        // Under fallback2A
        BT::ConditionHandlingRequest* con_handling_request = new BT::ConditionHandlingRequest("con hand");
        BT::SequenceNodeWithMemory* sequence3A = new BT::SequenceNodeWithMemory("seq 3A");

        // Under fallback2B
        BT::SequenceNodeWithMemory* sequence3B1 = new BT::SequenceNodeWithMemory("seq 3B1");
        BT::SequenceNodeWithMemory* sequence3B2 = new BT::SequenceNodeWithMemory("seq 3B2");

        // Level 4
        // Under sequence3A
        BT::FallbackNodeWithMemory* fallback4A1 = new BT::FallbackNodeWithMemory("fall 4A1");
        BT::ActionCheckDatabase* actioncheck_data = new BT::ActionCheckDatabase("Act Check Data");
        BT::FallbackNodeWithMemory* fallback4A2 = new BT::FallbackNodeWithMemory("fall 4A2");
        BT::ActionUpdateOrder* action_order = new BT::ActionUpdateOrder("act order");

        // Under sequence3B1
        BT::ConditionShoppinglist* con_shoppinglist = new BT::ConditionShoppinglist("con shop");
        BT::FallbackNode* fallback4B1A = new BT::FallbackNode("fall 4B1A");
        BT::ActionPick* action_pick_product = new BT::ActionPick("Act Pick Product");
        BT::ActionUpdateNextProduct* action_update_next_product = new BT::ActionUpdateNextProduct("Act Update");
        BT::ActionPickBasket* action_pick_basket = new BT::ActionPickBasket("act pick bask");

        // Under sequence3B2
        BT::ConditionRightArmState* condition_right_arm_state = new BT::ConditionRightArmState("Con R-Arm state");
        BT::FallbackNode* fallback4B2A = new BT::FallbackNode("fall 4B2A");
        BT::ActionDropBasket* action_drop_basket = new BT::ActionDropBasket("Act Drop Basket");

        // Level 5
        // Under fallback4A1
        BT::ActionInputRequest* action1_input= new BT::ActionInputRequest("Act Input");
        BT::ConditionInputRequest* condition_input = new BT::ConditionInputRequest("Con Input");

        // Under fallback4A2
        BT::ConditionDatabase1* condition_database1= new BT::ConditionDatabase1("Con Data 1");
        BT::SequenceNodeWithMemory* sequence5A = new BT::SequenceNodeWithMemory("seq 5A");


        // Under fallback4B1A
        BT::ConditionMoveBase* condition_move_base = new BT::ConditionMoveBase("Con Move");
        BT::ActionMoveBase* action_move_base = new BT::ActionMoveBase("Act Move");

        // Under fallback4B2A
        BT::ConditionReturnToHome* condition_move_home = new BT::ConditionReturnToHome("Con Home");
        BT::ActionReturnToHome* action_move_home = new BT::ActionReturnToHome("Act Home");


        // Level 6
        // Under sequence5A
        BT::ConditionDatabase2* condition_database2= new BT::ConditionDatabase2("Con Data 2");
        BT::ActionContinueRequest* action_continue = new BT::ActionContinueRequest("Act Contin");

        /****** Connecting the Nodes ******/
        // Top Branch
        sequence1->AddChild(fallback2A);
        sequence1->AddChild(fallback2B);

        // Branch A
        fallback2A->AddChild(con_handling_request);
        fallback2A->AddChild(sequence3A);

        sequence3A->AddChild(fallback4A1);
        sequence3A->AddChild(actioncheck_data);
        sequence3A->AddChild(fallback4A2);
        sequence3A->AddChild(action_order);
        sequence3A->AddChild(action_pick_basket);

        fallback4A1->AddChild(condition_input);
        fallback4A1->AddChild(action1_input);

        fallback4A2->AddChild(condition_database1);
        fallback4A2->AddChild(sequence5A);

        sequence5A->AddChild(condition_database2);
        sequence5A->AddChild(action_continue);

        //Branch B
        fallback2B->AddChild(sequence3B1);
        fallback2B->AddChild(sequence3B2);

        // Subbranch B1
        sequence3B1->AddChild(con_shoppinglist);
        sequence3B1->AddChild(fallback4B1A);
        sequence3B1->AddChild(action_pick_product);
        sequence3B1->AddChild(action_update_next_product);

        fallback4B1A->AddChild(condition_move_base);
        fallback4B1A->AddChild(action_move_base);

        // Subbranch B2
        sequence3B2->AddChild(condition_right_arm_state);
        sequence3B2->AddChild(fallback4B2A);
        sequence3B2->AddChild(action_drop_basket);

        fallback4B2A->AddChild(condition_move_home);
        fallback4B2A->AddChild(action_move_home);



        Execute(sequence1, TickPeriod_milliseconds);  // from BehaviorTree.cpp
    }
    catch (BT::BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

    return 0;
}
