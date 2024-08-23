#include "go.h"
void go()
{
    if (!needToOutGarage)
        outGarageFlag = 1;
    statusReset();
    binaryAlgorithm();
    /****************************ͼ����Ϣ��ȡ********************************/
    uint8 baseMid = myCarMid;
    baseMid = LAST_II.midline[5];
    // Ѱ��ɨ�ߺ���ͼ���е���
    if (IF.fork == FL || IF.crossroad == UL1 || IF.crossroad == UL2 || IF.crossroad == UL3)
        II.searchLineMid = getSearchLineColMid_oneSide(myCarMid + 5, 15, goLeft);
    else if (IF.fork == FR || IF.crossroad == UR1 || IF.crossroad == UR2 || IF.crossroad == UR3)
        II.searchLineMid = getSearchLineColMid_oneSide(myCarMid - 5, 15, goRight);
    else if (IF.garage == GR1 || IF.garage == GR2)
        II.searchLineMid = getSearchLineColMid_oneSide(myCarMid, 25, goRight);
    else if (IF.garage == GL1 || IF.garage == GL2)
        II.searchLineMid = getSearchLineColMid_oneSide(myCarMid, 25, goLeft);
    else if (IF.crossroad == CL1 || IF.crossroad == CL2)
        II.searchLineMid = getSearchLineColMid_oneSide(myCarMid - 5, 25, goRight);
    else if (IF.crossroad == CR1 || IF.crossroad == CR2)
        II.searchLineMid = getSearchLineColMid_oneSide(myCarMid + 5, 25, goLeft);
    else if (IF.ramp > 1)
    {
        baseMid = LAST_II.midline[0];
        II.searchLineMid = getSearchLineColMid(baseMid, 3);
    }
    else if (IF.annulus == AL2)
        II.searchLineMid = getSearchLineColMid_oneSide(baseMid, 15, goLeft);
    else if (IF.annulus == AR2)
        II.searchLineMid = getSearchLineColMid_oneSide(baseMid, 15, goRight);
    else
    {
        II.searchLineMid = getSearchLineColMid(baseMid, RANGE_GETMID);
        uint8 row = getMapYMax_Col(II.searchLineMid, allmap, PIXEL_WHITE);
        if (row == YM)
        {
            II.searchLineMid = getSearchLineColMid(baseMid, XX);
        }
    }
    /**************  ����������ͨ��basemap  ***************/
    uint8 mid = II.searchLineMid;
    for (int i = 0; i < XM; ++i) // �ױ��
    {
        if (mid >= i && allmap[0][mid - i] == PIXEL_WHITE) // ���ڲ���������
        {
            searchimg(mid - i, 0); // ��basemap�׵�͸�ֵinsidemap
            break;
        }
        if (mid + i < XM && allmap[0][mid + i] == PIXEL_WHITE)
        {
            searchimg(mid + i, 0);
            break;
        }
    }
    if (IF.annulus || IF.garage)
    {
        uint8 down = getDown();
        II.step = down < STEP1 ? down : STEP1;
    }
    else
        II.step = STEP1;
    searchLeftAndRightMap();
    // ��basemap�����Ͻ��к���ɨ��
    searchLines(II.searchLineMid);
    // ��ɨ�߷ֶΣ�����ɨ��
    getLineInfoLeft();
    getLineInfoRight();
    // ɨ�ߵĹؼ���
    if (II.bnum_all)
    {
        if (!(IF.ramp))
        {
            getLeftDownCorner();
            getRightDownCorner();
        }
        Get_insideMap();
        getSpeedTop();
    }
    if (!outGarageFlag && II.bnum_all)
    {
        IF.garage = twentyCmOutGarageStateMachine();
    }
    else if (outGarageFlag)
    {
        if (!IF.annulus && !IF.ramp && !IF.fork)
            IF.garage = twentyCmEnterGarageStateMachine();
        if ((!IF.garage || ((IF.garage == GL1 || IF.garage == GR1) && !startAddLine_enterGarage)) && !IF.annulus && !IF.ramp && !IF.fork && !IF.annulusDelay)
            IF.crossroad = twentyCmGoCrossroadStateMachine();
        if ((!IF.crossroad || IF.crossroad == CM2) && !IF.garage && !IF.annulus && !IF.fork && !IF.annulusDelay)
            IF.ramp = twentyCmGoRampStateMachine();
        if ((IF.crossroad == 0 || IF.crossroad == CL1 || IF.crossroad == CR1) && !IF.garage && !IF.ramp && !IF.fork)
            IF.annulus = twentyCmGoAnnulusStateMachine();
        if (!IF.garage && !IF.ramp && !IF.annulus && !IF.crossroad)
            IF.fork = twentyCmGoForkStateMachine();
    }
    bodyworkSafetyAssess();
    deleteline();
    mergeAndFix();
    regetSpeedTop();
    bodyworkSafetyAssess();
    if (runState == RUNNING && isOut() && !IF.garage)
    {
        runState = BRAKING;
        stopReason = RunOutLine;
    }
    if (IF.garage != 253 && runState != BRAKING)
        directionControl();
    drivingRecorder_record();
}
