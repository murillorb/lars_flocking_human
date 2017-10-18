#ifndef VSSSTRATEGY_WR2016_GAMEONA_H
#define VSSSTRATEGY_WR2016_GAMEONA_H

#include <WRCoach/entity/controlmodule/coach/strategy/strategystate.hh>

class VSSStrategy_WR2016_GameOnA : public StrategyState {
private:
    // Playbooks
    VSSPlaybook_AttackA *_pb_attackA;
    VSSPlaybook_Defense *_pb_defense;
    VSSPlaybook_Goal *_pb_goal;

    void configure(int numOurPlayers);
    void run(int numOurPlayers);
public:
    VSSStrategy_WR2016_GameOnA();
    QString name();
};

#endif // VSSSTRATEGY_WR2016_GAMEONA_H
